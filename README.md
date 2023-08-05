# flexicon_cosmos
This repository contains a GNU Radio out-of-tree (OOT) module to support the second-generation (Gen-2) full-duplex (FD) hardware that has been developed by the Columbia University [FlexICoN project](https://flexicon.ee.columbia.edu). 

For details on the FD radio testbed publicly available to researchers on the COSMOS PAWR platform, please review the [paper](https://wimnet.ee.columbia.edu/wp-content/uploads/2021/10/orbit_cosmos_comnets_2021.pdf) describing the testbed integration, as well as the [tutorial](https://wiki.cosmos-lab.org/wiki/Tutorials/Wireless/FullDuplex) explaining how to use the FD radios in the testbed.

The OOT module provided in this repository is developed for GNU Radio 3.8. We also recommend the installation of the [gr-ieee802-11](https://github.com/bastibl/gr-ieee802-11) OOT module which provides an effective means of packetized data transmission.

If you use this code or the COSMOS testbed, please cite the following paper:

M. Kohli, T. Chen, M. Baraani Dastjerdi, J. Welles, I. Seskar, H. Krishnaswamy, and G. Zussman, “Open-access full-duplex wireless in the ORBIT and COSMOS testbeds,” *Computer Networks*, no. 199, p. 108420, Aug. 2021.

## Dependencies
This OOT module requires some additional dependencies to successfully build. Please ensure you install Eigen3 (libeigen3-dev) as well as the [SUB-20 driver](http://www.xdimax.net/download/SUB-20-snap-130926.tgz). The SUB-20 driver can be ignored if the `fde_config` and `sub20_init` blocks are disabled.  
