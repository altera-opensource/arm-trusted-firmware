# Intel SOCFPGA Documentation for Trusted Firmware-A

## Introduction

This README file describes TF-A support for Intel SoCFPGA.
For detailed information regarding the hardware product, please refer to
https://www.intel.com/content/www/us/en/products/programmable/soc.html

----

## Table of Contents

1. [Release Information](#1-release-information)
2. [Device Family Support and Compatibility](#2-device-family-support-and-compatibility)
3. [Feature Support](#3-tf-a-feature-support)
4. [Major Changes and Known Issues](#4-major-changes)
5. [Known Issues and Limitation](#5-known-issues-and-limitation)

----

## 1. Release Information

Version		|	Release Branch		|	TF-A Tag
-------		|	--------------		|	--------
Current (N)	|	socfpga_v2.4.0		|	v2.4
N - 1		|	socfpga_v2.3		|	v2.3
N - 2		|	socfpga_v2.1		|	v2.1

----

## 2. Device Family Support and Compatibility

This section details SoCFPGA Device Family supported by TF-A and its underlying processor microarchitecture.
Table below keeps track of Quartus version tested to be compatible with current TF-A release.


SOCFPGA Device Family	|	Processor Microarchitecture	|	Quartus Prime Pro Edition
---------------------	|	---------------------------	|	-------------------------------
Stratix 10		|	Quad-core ARM Cortex-A53	|	20.1, 20.2, 20.3, 20.4
Agilex			|	Quad-core ARM Cortex-A53	|	20.1, 20.2, 20.3, 20.4
Diamond Mesa		|	Quad-core ARM Cortex-A53	|	Early Access

----

## 3. TF-A Feature Support

Hardware Feature		|	Stratix 10	|	Agilex		|	Diamond Mesa
----------------		|	----------	|	------		|	------------
SDRAM				|	Yes		|	Yes		|	No
HPS Bridge (LWH2F, H2F, F2S)	|	Yes		|	Yes		|	No
HPS Cold/Warm Reset		|	Yes		|	Yes		|	Yes
HPS Linux SMP Boot		|	Yes		|	Yes		|	Yes
Ethernet (EMAC controller)	|	Yes		|	Yes		|	No
Synopsys UART controller	|	Yes		|	Yes		|	No
Synopsys Watchdog timer		|	Yes		|	Yes		|	No
Synopsys SDMMC controller	|	Yes		|	Yes		|	No
Cadence QSPI controller		|	Yes		|	Yes		|	No
Denali NAND controller		|	No		|	No		|	No
Synopsys GPIO controller	|	No		|	No		|	No
Synopsys USB controller		|	No		|	No		|	No
Synopsys I2C master controller	|	No		|	No		|	No

----

FPGA Feature			|	Stratix 10	|	Agilex		|	Diamond Mesa
------------			|	----------	|	------		|	------------
FPGA Configuration		|	Yes		|	Yes		|	No
Partial Reconfiguration		|	Yes		|	No		|	No
Remote System Update (RSU)	|	Yes		|	Yes		|	No
FPGA Crypto Service (FCS)	|	No		|	Yes<sup>1</sup>	|	Yes<sup>1</sup>

----

### Notes
<sup>1</sup> Refer to section 5.3: FCS Limitation

----

## 4. Major Changes

1. Based on TF-A version 2.4 official release
	- Refer to docs/change-log.rst for details

2. Update FCS internal logic to align with latest firmware
	- Decryption command now sends Owner ID as part of the direct argument
	- Provision data now includes additional Provision Status field
	- Configuration Status now based on the latest configuration request type
	- Configuration type flags now dealt with bit-wise

----

## 5. Known Issues and Limitation

1. Uboot + TF-A Boot Flow
	- Supported in altera-opensource/u-boot-socfpga branch:socfpga_v2020.04 onwards

2. TF-A Diamond Mesa Support
	- Current release only have support for BL31
	- Hence TF-A only usable as secure monitor for Diamond Mesa platform

3. FPGA Crypto Service (FCS) Limitation
	- Not supported in Intel Quartus Prime Pro 20.4 release
	- Only supported in Simics software virtual platform

4. Vendor Authorized Boot (VAB)
	- Not supported for TF-A + UEFI boot flow for this release
	- BL2 -> BL31 -> UEFI -> Linux

5. Double-Bit Error Handling in EL3
	- Linux EDAC framework failure in triggering DBE handling is reported
	- Processor cores are trapped in EL3
	- Fixes are delayed to next release cycle
