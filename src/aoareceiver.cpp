/* 
 * Copyright 2019 Marco Cominelli
 * Copyright 2019 Francesco Gringoli
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <uhd/types/tune_request.hpp>
#include <uhd/utils/thread_priority.hpp>
#include <uhd/utils/safe_main.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <boost/program_options.hpp>
#include <boost/format.hpp>
#include <boost/thread.hpp>
#include <boost/lexical_cast.hpp>
#include <csignal>
#include <boost/algorithm/string.hpp>
#include <iostream>
#include <complex>
#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <unistd.h>

#define XOR(x,y) ((x && !y) || (!x && y))

namespace po = boost::program_options;

volatile static bool stop_signal = false;
void sigint_handler(int) {stop_signal = true;}

volatile static int counter = 0;

pthread_spinlock_t lock[2];
volatile size_t bankA_nsamps, bankB_nsamps;

struct proc_pars_t {
    std::vector<std::complex<float> *> bankA_ptrs;
    std::vector<std::complex<float> *> bankB_ptrs;
    size_t samps_per_buff;
};

typedef struct MyData {
    size_t sn;
    float aoa;
    float amplitude[2];
} mydata_t;

typedef struct Command {
    int type;
    int value;
} cmd_t;

volatile int do_print = 0;
bool do_hop = false;
bool change_gain = false;
double gain, freq;
unsigned int freq_inc = 0;

/*
 * Useful commands for running experiments.
 * (type corresponding letters while program is running)
 */
void* keyboard_routine(void* pars)
{
    while (stop_signal == false) {
        int c = fgetc(stdin);
        if (c == 'p') {          // print 1 pkt info to file
            do_print = 1;
        } else if (c == 'g') {   // adjust gain
            change_gain = true;
        } else if (c == 'h') {   // hop to next channel (0 to 39)
            do_hop = true;
        } else if (c == 'r') {   // run complete capture
            sleep(4);
            printf("\a");  // beep alert (start)
            for (int kk = 0; kk < 40; kk++) {
                do_hop = true;
                sleep(1);
                do_print = 30;
                while(do_print > 0) usleep(5000);
            }
            printf("\a");  // beep alert (stop)
        }
        usleep(1000);
    }

    return NULL;
}


inline void dewhiten(uint8_t *pdu, unsigned int length)
{
    /* Scheme of the LFSR used for dewhitening
     *
     *   7   6   5   4   3   2   1   0
     * +---+---+---+---+---+---+---+---+
     * | x | 1 | c | c | c | c | c | c |
     * +---+---+---+---+---+---+---+---+
     *
     * Shifting right every tick.
     */
    uint8_t lfsr = 0x40 | (0x3f & 22); // Channel 22 hardcoded
    for (unsigned int byte = 0; byte < length; byte++) {
        for (int b = 0x01; b <= 0x80; b <<=1) {
            if (lfsr & 0x01) {
                pdu[byte] ^= b;
                lfsr ^= 0x88;
            }
            lfsr >>= 1;
        }
    }
}


uint32_t compute_crc(uint8_t *pdu, int len, uint32_t init_val)
{
    uint32_t crc = init_val;
    while (len--) {
        for (int i = 0x01; i <= 0x80; i <<= 1) {
            uint32_t crc_bit_is_one = crc & 0x00000001;
            uint32_t pdu_bit_is_one = *pdu & i;
            if ( XOR(crc_bit_is_one,pdu_bit_is_one) ) {
                crc ^= 0x01B4C000;
            }
            crc >>= 1;
        }
        pdu++;
    }
    return crc;
}


void *process_routine(void *pars)
{
    struct proc_pars_t *procpars = (struct proc_pars_t *) pars;
    size_t kk = 0, select_bank;
    size_t samps_per_buff = procpars->samps_per_buff;
    size_t circbuf_size = 2 * samps_per_buff;
    std::vector<std::complex<float> *> curr_bank;
    float *phasebuf0 = (float *) malloc(circbuf_size * sizeof(float));
    float *phasebuf1 = (float *) malloc(circbuf_size * sizeof(float));
    uint8_t *binbuf0 = (uint8_t *) malloc(circbuf_size * sizeof(uint8_t));
    float *amplbuf0 = (float *) malloc(circbuf_size * sizeof(float));
    size_t opposite_bank;
    const unsigned int srate = 2;
    std::vector<unsigned int> snarray;
    unsigned int sn;
    float dphase;
    size_t idx;

    float *pktphase0 = (float *) malloc(128 * sizeof(float));
    float *pktphase1 = (float *) malloc(128 * sizeof(float));
    
    mydata_t mydata;
    float aoa_ma = 0;

    FILE * fptr = fopen("angles.dat", "a");

    while (stop_signal == false) {
        select_bank = kk++ % 2;
        // Acquire buffer.
        if (select_bank == 0) curr_bank = procpars->bankA_ptrs;
        else curr_bank = procpars->bankB_ptrs;
        pthread_spin_lock(&lock[select_bank]);

        std::complex<float> *buff0_ptr = curr_bank[0];
        std::complex<float> *buff1_ptr = curr_bank[1];

        // Compute phase and binary values
        for (int i = 0; i < samps_per_buff; i++) {
            idx = i + samps_per_buff * select_bank;

            // Compute phase on both channels.
            phasebuf0[idx] = atan2f(buff0_ptr[i].imag(), buff0_ptr[i].real());
            phasebuf1[idx] = atan2f(buff1_ptr[i].imag(), buff1_ptr[i].real());

            // Compute amplitude (only channel 0).
            amplbuf0[idx] = buff0_ptr[i].real() * buff0_ptr[i].real() +
                            buff0_ptr[i].imag() * buff0_ptr[i].imag();

            // Compute phase difference (only channel 0).
            dphase = (idx != 0) ? phasebuf0[idx] - phasebuf0[idx-1]
                                : phasebuf0[0] - phasebuf0[circbuf_size-1];

            // Handle phase wrapping.
            if (dphase > M_PI) dphase -= 2 * M_PI;
            else if (dphase < -M_PI) dphase += 2 * M_PI;

            // Discriminate bits (only channel 0).
            binbuf0[idx] = (dphase > 0) ? 1 : 0;
        }
        
        opposite_bank = (select_bank + 1) % 2;

        sn = 0;

        for (int i = 0; i < samps_per_buff; i++) {
            idx = i + samps_per_buff * opposite_bank;
            uint8_t transitions = 0;
            // Detect preamble.
            for (int c = 0; c < 8 * srate; c += srate) {
                if (binbuf0[(idx+c+srate)%circbuf_size] > binbuf0[(idx+c)%circbuf_size]) {
                    transitions++;
                }
            }
            if (transitions != 4) continue; // not a preamble

            uint32_t aa = 0x00;
            uint32_t offset = idx + 8 * srate;
            for (unsigned int c = 0; c < 32 * srate; c += srate) {
                // note: (c >> 1) = (c/srate) since srate = 2
                aa |= binbuf0[(offset + c) % circbuf_size] << (c >> 1);
            }
            if (aa != 0x8e89bed6) continue; // not our aa

            // Extract packet length
            offset = idx + (8+32) * srate;

            // Extract packet PDU.
            uint8_t length = 30+2+3;
            uint8_t pdu[128] = {0x00};
            for (int byte = 0; byte < length; byte++)
                for (int b = 0; b < 8; b++)
                    pdu[byte] |= binbuf0[(offset+(byte*8+b)*srate)%circbuf_size] << b;

            dewhiten(pdu, length);

            uint32_t crc_packet = 0, crc_computed = 0;

            for (int byte=0; byte<3; byte++) {
                int bitpos = 0;

                for (int b = 0x01; b <= 0x80; b <<= 1) {
                    if (pdu[32+byte] & b) {
                        crc_packet |= 0x01 << (byte*8+bitpos);
                    }
                    bitpos++;
                }
            }
            
            crc_computed = compute_crc(pdu, 32, 0x00AAAAAA);
            if (crc_computed != crc_packet) continue; // Discard wrong CRCs.

            sn = pdu[15] | (pdu[14] << 8) | (pdu[13] << 16) | (pdu[12] << 24);

            for (int n=0; n<128 ; n++) {
                pktphase0[n] = phasebuf0[(idx+(8+32+16+8)*2+n)%circbuf_size];
                pktphase1[n] = phasebuf1[(idx+(8+32+16+8)*2+n)%circbuf_size];
            }

            break;
        }

        if (sn != 0 && sn >> 20 == 0) {
            float unwphase0[128];
            float unwphase1[128];
            float avg_energy = 0;
            float dphasetmp;

            // Unwrap phase
            unwphase0[0] = pktphase0[0];
            unwphase1[0] = pktphase1[0];
            for (int n=1; n<128; n++) {
                // Channel 0
                dphasetmp = pktphase0[n]-pktphase0[n-1];
                if (dphasetmp > M_PI) dphasetmp -= 2 * M_PI;
                else if (dphasetmp < -M_PI) dphasetmp += 2 * M_PI;
                unwphase0[n] = unwphase0[n-1] + dphasetmp;
                // Channel 1
                dphasetmp = pktphase1[n]-pktphase1[n-1];
                if (dphasetmp > M_PI) dphasetmp -= 2 * M_PI;
                else if (dphasetmp < -M_PI) dphasetmp += 2 * M_PI;
                unwphase1[n] = unwphase1[n-1] + dphasetmp;
            }

            float phasediff = 0, phasediff_emulated = 0;
            float amplitude = 0;
            float phaseinc = 0;

            for (int n = 0; n < 8; n++) phaseinc += (unwphase0[64+n] - unwphase0[64+n-1]);
            phaseinc /= 8;   // average phase increment
            phasediff_emulated = unwphase0[71] + 2*phaseinc - unwphase1[73];

            for (int n = 0; n < 128; n++) {
                phasediff += (unwphase0[n]-unwphase1[n])/128;
                amplitude += amplbuf0[(idx+n)%circbuf_size] / 128;
            }

            phasediff = fmodf(phasediff, 2 * M_PI);
            phasediff_emulated = fmodf(phasediff, 2 * M_PI);

            // Handle phase wrapping.
            if (phasediff > M_PI) phasediff -= 2 * M_PI;
            else if (phasediff < -M_PI) phasediff += 2 * M_PI;

            // Handle phase wrapping (emulated phase).
            if (phasediff_emulated > M_PI) phasediff_emulated -= 2 * M_PI;
            else if (phasediff_emulated < -M_PI) phasediff_emulated += 2 * M_PI;
                

            float lambda = 300e6/freq;
            const float d = 0.06;
            mydata.aoa = acos((lambda * phasediff)/(2 * M_PI * d));
            mydata.aoa = mydata.aoa / M_PI * 180;
            mydata.sn = sn;
            mydata.amplitude[0] = amplitude;

            if (isnan(mydata.aoa) == 0) {
                printf("\rsn: %8lu, aoa: %3.0f, ampl: %8.6f, phasediff %8.6f ",
                    mydata.sn, mydata.aoa, amplitude, phasediff_emulated);
                fflush(stdout);

                if (do_print > 0) {
                    fprintf(fptr, "%d %f %f %f %f %f %u\n",
                        counter,
                        mydata.aoa,
                        phasediff,
                        phasediff_emulated,
                        amplitude,
                        gain,
                        freq_inc/1000000 + 2400);
                    do_print--;
                    counter++;
                }
            }
        }

        // Release buffer
        if (select_bank == 0) bankA_nsamps = 0;
        else bankB_nsamps = 0;
        
        pthread_spin_unlock(&lock[select_bank]);
    }
    
    fclose(fptr);

    free(amplbuf0);
    free(pktphase0);
    free(pktphase1);
    free(binbuf0);
    free(phasebuf0);
    free(phasebuf1);

    return NULL;
}


int UHD_SAFE_MAIN(int argc, char *argv[])
{
    uhd::set_thread_priority_safe();

    // Variables to be set by program options.
    std::string args, channel_list, subdev;
    double rate;
    size_t total_num_samps;

    // Setup program options.
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help",
            "print this help message")
        ("args", po::value<std::string>(&args)->default_value(""),
            "USRP device arguments")
        ("channels", po::value<std::string>(&channel_list)->default_value("0,1"),
            "set channels to use (e.g. \"0\", \"0,1\")")
        ("freq", po::value<double>(&freq)->default_value(2450e6),
            "set the RF center frequency (Hz)")
        ("gain", po::value<double>(&gain)->default_value(30),
            "set RF gain of the receiving chains")
        ("nsamps", po::value<size_t>(&total_num_samps)->default_value(10e3),
            "set total number of samples to receive")
        ("rate", po::value<double>(&rate)->default_value(2e6),
            "set the sampling rate (samples per second)")
        ("subdev", po::value<std::string>(&subdev)->default_value("A:A A:B"),
            "set frontend specification")
    ;
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    // Print help message;
    if (vm.count("help")) {
        std::cout << "User positioning demo using AoA features" << std::endl;
        std::cout << desc << std::endl;
        return ~0;
    }

    // Create a USRP device.
    std::cout << "Creating USRP device..." << std::endl;
    uhd::usrp::multi_usrp::sptr usrp = uhd::usrp::multi_usrp::make(args);
    std::cout << std::endl;

    // Select the RX subdevice first; this mapping affects all the settings.
    if (vm.count("subdev")) usrp->set_rx_subdev_spec(subdev);

    // Set the rx sample rate on all channels.
    std::cout << boost::format("Setting RX rate to %f MS/s\n")
                % (rate/1e6);
    usrp->set_rx_rate(rate);
    std::cout << boost::format("Actual RX rate: %f MS/s.\n\n")
                % (usrp->get_rx_rate()/1e6);

    // Lock motherboard clock to internal reference source and reset time register.
    usrp->set_clock_source("internal");
    usrp->set_time_now(uhd::time_spec_t(0.0));
    std::cout << "Device timestamp set to 0.0 s." << std::endl;

    // Set the rx center frequency on all channels.
    usrp->set_rx_freq(freq, 0);
    usrp->set_rx_freq(freq, 1);
    std::cout << boost::format("Center frequency set to %f MHz.\n")
                % (usrp->get_rx_freq()/1e6);

    // Set the rx gain on all channels.
    usrp->set_rx_gain(gain, 0);
    usrp->set_rx_gain(gain, 1);
    std::cout << boost::format("Gain set to %.1f dB.\n\n")
                 % (usrp->get_rx_gain());

    // Set the rx antennas.
    usrp->set_rx_antenna("TX/RX", 0);
    usrp->set_rx_antenna("TX/RX", 1);

    // Select rx channels.
    std::vector<std::string> channel_strings;
    std::vector<size_t> channel_nums;
    boost::split(channel_strings, channel_list, boost::is_any_of("\"',"));
    for (size_t c = 0; c < channel_strings.size(); c++) {
        size_t chan = boost::lexical_cast<int>(channel_strings[c]);
        if (chan < usrp->get_rx_num_channels()) {
            channel_nums.push_back(boost::lexical_cast<int>(channel_strings[c]));
        } else {
            throw std::runtime_error("Invalid channel(s) specified.");
        }
    }

    // Create a receive streamer.
    // It linearly maps channels (index0 = channel0, index1 = channel1, ...)
    uhd::stream_args_t stream_args("fc32", "sc16");
    stream_args.channels = channel_nums;
    uhd::rx_streamer::sptr rx_stream = usrp->get_rx_stream(stream_args);

    // Setup streaming.
    std::cout << "Begin streaming samples... ";
    uhd::stream_cmd_t stream_cmd(uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS);
    stream_cmd.num_samps = total_num_samps;
    stream_cmd.stream_now = false;
    stream_cmd.time_spec = uhd::time_spec_t(2.0);
    rx_stream->issue_stream_cmd(stream_cmd);

    // Allocate receiver buffers.
    // There are two banks of memory containing one register per buffer.
    const size_t samps_per_buff = rx_stream->get_max_num_samps();
    std::vector<std::vector<std::complex<float>>> bankA_buffs(
        usrp->get_rx_num_channels(),
        std::vector<std::complex<float>> (samps_per_buff)
    );
    std::vector<std::vector<std::complex<float>>> bankB_buffs(
        usrp->get_rx_num_channels(),
        std::vector<std::complex<float>> (samps_per_buff)
    );

    // Create a vector of pointers to each buffer.
    std::vector<std::complex<float> *> bankA_ptrs;
    std::vector<std::complex<float> *> bankB_ptrs;
    for (size_t i = 0; i < bankA_buffs.size(); i++) {
        bankA_ptrs.push_back(&bankA_buffs[i].front());
        bankB_ptrs.push_back(&bankB_buffs[i].front());
    }
    
    uhd::rx_metadata_t md;     // metadata will be filled in by recv()
    double timeout = 10.0;     // timeout for recv()
    size_t num_acc_samps = 0;  // number of accumulated samples

    size_t kk = 0;
    size_t select_bank;
    size_t num_rx_samps;
    std::vector<std::complex<float> *> curr_bank;

    // Initialise the spinlocks.
    pthread_spin_init(&lock[0], PTHREAD_PROCESS_PRIVATE);
    pthread_spin_init(&lock[1], PTHREAD_PROCESS_PRIVATE);

    // Create processing thread and keyboard input thread.
    pthread_t proc_thread;
    pthread_t keyboard_thread;
    struct proc_pars_t proc_pars = {bankA_ptrs, bankB_ptrs, samps_per_buff};
    
    pthread_create(&proc_thread, NULL, process_routine, &proc_pars);
    pthread_create(&keyboard_thread, NULL, keyboard_routine, NULL);

    // Set SIGINT for stopping execution.
    std::signal(SIGINT, &sigint_handler);
    std::cout << "Press Ctrl + C to stop streaming." << std::endl;

    // To transmitter.
    int txsock = 0;
    struct sockaddr_in txaddr;
    if ((txsock = socket(AF_INET, SOCK_DGRAM, 0)) < 0) stop_signal = true;
    memset (&txaddr, '0', sizeof(txaddr));
    txaddr.sin_family = AF_INET;
    txaddr.sin_port = htons(8082);

    // To other receiver.
    int slavesock = 0;
    struct sockaddr_in slaveaddr;
    if ((slavesock = socket(AF_INET, SOCK_DGRAM, 0)) < 0) stop_signal = true;
    memset (&slaveaddr, '0', sizeof(slaveaddr));
    slaveaddr.sin_family = AF_INET;
    slaveaddr.sin_port = htons(8081);

    if (inet_pton(AF_INET, "192.168.0.3", &txaddr.sin_addr) <= 0) stop_signal = true;
    if (inet_pton(AF_INET, "192.168.0.2", &slaveaddr.sin_addr) <= 0) stop_signal = true;

    while (stop_signal == false) {
        // Select current bank.
        select_bank = kk++ % 2;

        if (select_bank == 0) curr_bank = bankA_ptrs;
        else curr_bank = bankB_ptrs;
        pthread_spin_lock(&lock[select_bank]);

        // Receive one block of data per channel.
        num_rx_samps = rx_stream->recv(
            curr_bank, samps_per_buff, md, timeout, false
        );

        // Throw on all errors.
        if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_OVERFLOW) {
            // do nothing
        }
        else if (md.error_code != uhd::rx_metadata_t::ERROR_CODE_NONE) {
            std::cout << boost::format("\nReceived %u samples.\n") % num_acc_samps;
            throw std::runtime_error(md.strerror());
        }

        num_acc_samps += num_rx_samps;
        
        if (select_bank == 0) bankA_nsamps = num_rx_samps;
        else bankB_nsamps = num_rx_samps;
        pthread_spin_unlock(&lock[select_bank]);

        if (change_gain) {
            gain = fmodf(gain+10,60);
            usrp->set_rx_gain(gain, 0);
            usrp->set_rx_gain(gain, 1);
            std::cout << boost::format("Gain set to %.1f dB.\n") % (usrp->get_rx_gain());
            change_gain = false;

            cmd_t command = {1, (int) gain};
            sendto(slavesock, &command, sizeof(command), 0, (struct sockaddr *)&slaveaddr, sizeof(slaveaddr));
        }

        if (do_hop) {
            freq = (float) ((freq_inc) % 80000000) + 2402e6;
            freq_inc += 2000000;
            usrp->set_rx_freq(freq, 0);
            usrp->set_rx_freq(freq, 1);
            std::cout << boost::format("Frequency set to %f MHz.\n") % (usrp->get_rx_freq()/1e6);
            do_hop = false;

            int freq_integer = 2400 + freq_inc/1000000;

            cmd_t command = {2, freq_integer};
            sendto(txsock, &freq_integer, sizeof(freq_integer), 0, (struct sockaddr*)&txaddr, sizeof(txaddr)); 
            sendto(slavesock, &command, sizeof(command), 0, (struct sockaddr *)&slaveaddr, sizeof(slaveaddr));
        }
    }

    pthread_join(proc_thread, NULL);
    pthread_cancel(keyboard_thread);

    pthread_spin_destroy(&lock[0]);
    pthread_spin_destroy(&lock[1]);
    std::cout << boost::format("\nReceived %u samples.\n") % num_acc_samps;
    std::cout << "Done." << std::endl;

    return EXIT_SUCCESS;
}

