# flexicon_cosmos
This repository contains a GNU Radio out-of-tree (OOT) module to support the second-generation (Gen-2) full-duplex (FD) hardware that has been developed by the Columbia University [FlexICoN project](https://flexicon.ee.columbia.edu). 

For details on the FD radio testbed publicly available to researchers on the COSMOS PAWR platform, please review the [paper](https://wimnet.ee.columbia.edu/wp-content/uploads/2021/10/orbit_cosmos_comnets_2021.pdf) describing the testbed integration, as well as the [tutorial](https://wiki.cosmos-lab.org/wiki/Tutorials/Wireless/FullDuplex) explaining how to use the FD radios in the testbed.

The OOT module provided in this repository is developed for GNU Radio 3.8. We also recommend the installation of the [gr-ieee802-11](https://github.com/bastibl/gr-ieee802-11) OOT module which provides an effective means of packetized data transmission.

If you use this code or the COSMOS testbed, please cite the following paper:

M. Kohli, T. Chen, M. Baraani Dastjerdi, J. Welles, I. Seskar, H. Krishnaswamy, and G. Zussman, “Open-access full-duplex wireless in the ORBIT and COSMOS testbeds,” *Computer Networks*, no. 199, p. 108420, Aug. 2021.

## Dependencies
This OOT module requires some additional dependencies to successfully build. Please ensure you install Eigen3 (libeigen3-dev) as well as the [SUB-20 driver](http://www.xdimax.net/download/SUB-20-snap-130926.tgz). The SUB-20 driver can be ignored if the `fde_config` and `sub20_init` blocks are disabled.  

## Hardware
### The Gen-2 Wideband Frequency-Domain Equalization (FDE)-based RF Canceller Box
Please refer to our [Computer Networks paper](https://wimnet.ee.columbia.edu/wp-content/uploads/2020/08/wintech2020_orbit_cosmos_fullduplex_integration.pdf) for details on the integration of the Gen-2 RF Canceller Box in the COSMOS testbed. Our [MobiCom'19 paper](https://wimnet.ee.columbia.edu/wp-content/uploads/2018/12/FDE_MobiCom19.pdf) provides an in-depth evaluation of the Gen-2 hardware.

### Universal Software Radio Peripheral (USRP) 
Our full-duplex transceiver is based on the Ettus Research USRP X310.

### SUB-20 
The [SUB-20](http://www.xdimax.com/sub20/sub20.html) is a multi-interface USB adapter for providing standard interfaces between PC (USB host) and different hardware devices.  Specifically, we use the `SPI` module on the SUB-20 to control and configure our hardware. The user manual can be found [here](http://www.xdimax.com/sub20/doc/sub20-man.pdf). 

## Acknowledgements
This work was supported in part by NSF grants CCS-1547406, CNS-1827923, CNS-2148128, EEC-2133516, and DGE-2036197 ECCS-1547406 and CNS-1827923, NSF-BSF grant CNS-1910757, the DARPA RF-FPGA and SPAR programs, a Qualcomm Innovation Fellowship, Texas Instruments, Intel. We thank Steven Alfano, Jelena Diakonikolas, Aishwarya Rajen, Jinhui Song, Mingyan Yu, and Leoni Lu for their contributions to various aspects of the project. We thank Ivan Seskar, Jakub Kolodziejski, Michael Sherman, and Prasanthi Maddala from WINLAB, Rutgers University, for their help on the integration with the ORBIT and COSMOS testbeds. We also thank Neel Pandeya, Kira Theuer and Kendall Ruiz from NI and the NI technical support team for their help.
