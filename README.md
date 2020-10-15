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
Current (N)	|	socfpga_v2.3		|	v2.3
N - 1		|	socfpga_v2.1		|	v2.1

----

## 2. Device Family Support and Compatibility

This section details SoCFPGA Device Family supported by TF-A and its underlying processor microarchitecture.
Table below keeps track of Quartus version tested to be compatible with current TF-A release.


SOCFPGA Device Family	|	Processor Microarchitecture	|	Quartus Prime Pro Edition
---------------------	|	---------------------------	|	-------------------------------
Stratix 10		|	Quad-core ARM Cortex-A53	|	20.1, 20.3
Agilex			|	Quad-core ARM Cortex-A53	|	20.1, 20.3
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

1. Enable Uboot + TF-A Flow
	- New boot flow where TF-A's BL31 serves as secure monitor for U-boot
	- Reduce engineering effort for multiple bootloader projects by sharing SMC code base
	- Notably TF-A has better native support for PSCI implementations
	- Enable SPL entrypoint into BL31 and secondary CPUs handling during boot
	- SPL (EL3) -> TF-A BL31 (EL3) -> U-Boot Proper (EL2) -> Linux (EL1)
	- BL31 will be bundled as part of a FIT image
		1. U-Boot Proper (u-boot-nodtb.bin)
		2. U-Boot Proper DTB (u-boot.dtb)
		3. TF-A BL31 (bl31.bin)
	- SPL will be responsible for loading this FIT image


2. Add support for Intel Diamond Mesa
	- Initial support for Intel new platform codenamed Diamond Mesa (DM)
	- DM BL31 will share most of the common service provided in Agilex and Stratix 10

3. Enable support for FPGA Crypto Service (FCS)
	- FCS provides the capabilities of hardware Crypto IPs for customer usage
	- Linux Tools and API are provided to help interfacing with this service through BL31
	- TF-A BL31 will be the component handling HPS communication with SDM
	- Key features supported as part of this service as follows:
		1. Image Authentication
		2. Hardware Random Number Generator
		3. Encryption Service
		4. Decryption Service
		5. Provision Data Dump
	- Supported platform: Simics software virtual platform for Intel SoCFPGA 64bits
		(Agilex, Diamond Mesa)

4. Vendor Authorized Boot (VAB)
	- A secure boot feature currently supported in the new Uboot + TF-A bootflow
	- Takes advantage of Image Authentication capabilities of FCS
	- VAB ensures that each boot images are authenticated before execution
	- How it works:
		1. Security Certificate are generated for each boot images (.ccert)
		2. Certificates are signed then appended to the original image (signed_*)
		3. FIT images are generated from the signed boot images from earlier
		4. Each of these certificates will be authenticated during boot

5. Additional RSU features
	- In this new release, RSU functionalities are expanded to cover more use cases
	- List of these new SMC commands are as follows:
		1. RSU DCMF Version
		2. RSU DCMF Status
		3. RSU Max Retry

----

## 5. Known Issues and Limitation

1. Uboot + TF-A Boot Flow
	- Supported in altera-opensource/u-boot-socfpga branch:socfpga_v2020.04 onwards

2. TF-A Diamond Mesa Support
	- Current release only enable support for BL31
	- Hence TF-A only usable as secure monitor for Diamond Mesa platform

3. FPGA Crypto Service (FCS) Limitation
	- Not supported in Intel Quartus Prime Pro 20.3 release
	- Only supported in Simics software virtual platform

4. Vendor Authorized Boot (VAB)
	- Not supported for TF-A + UEFI boot flow for this release
	- BL2 -> BL31 -> UEFI -> Linux
