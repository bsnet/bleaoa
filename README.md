# bleaoa
## Tool for measuring Angle of Arrival with BLE5.1 on SDR

This demo software can be used to measure the accuracy of the Angle of Arrival (AoA) technique adopted in Bluetooth Low Energy (BLE) 5.1. The software runs on Software-Defined Radios (SDR) manufactured by Ettus Research.

## Components included

The programs included in this package run on systems that are connected to SDR platforms. The program running on a specific system depends on the role of the SDR in the experimental setup (transmitter or receiver).
In particular, the package includes:

* **aoatransmitter**: the transmitter sends BLE packets periodically (*beacons*). We have used this program with a USRP N200 SDR.
* **aoareceiver**: the receiver captures the *beacons* and process them in order to determine the AoA of the transmitter. We have used this program with a USRP B210 SDR.

The software has been tested on Ubuntu 18.04 and 16.04.

## Setup

Before building the programs, ensure that you have installed the USRP Open-Source Toolchain for Ubuntu (instructions [here](https://kb.ettus.com/Building_and_Installing_the_USRP_Open-Source_Toolchain_(UHD_and_GNU_Radio)_on_Linux), only UHD required).

Then, prepare the software for building using CMake:
```
    mkdir build
    cd build
    cmake ..
```

Finally, build all the software with
```
    make
```
or build only specific components by specifying the name. For example, to build only *aoareceiver* do:
```
    make aoareceiver
```

## Running the software

To run the software, first do ```mv``` to the *build* directory.

Programs can run with default parameter by executing:
```
    ./<program>
```
Arguments accepted by the programs can be shown with:
```
    ./<program> --help
```

The system running *aoareceiver* will display the AoA of the packets sent by *aoatransmitter*.
AoA is considered wrt. the axis going through the two antennas of the receiver.

## Publications

The code in this repository has been used in the following publications:

* M. Cominelli, P. Patras, F. Gringoli, [Dead on Arrival: An Empirical Study of The Bluetooth 5.1 Positioning System](https://dl.acm.org/doi/10.1145/3349623.3355475), *WiNTECH '19: Proceedings of the 13th International Workshop on Wireless Network Testbeds, Experimental Evaluation & Characterization*, October 2019
