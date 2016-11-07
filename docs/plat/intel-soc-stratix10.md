ARM Trusted Firmware for Intel SoC Stratix 10
=============================================

ARM Trusted Firmware implements the EL3 firmware layer for Intel
Stratix 10 SoC.
The platform only uses the runtime part of ATF as Intel SoC already has a
BootROM (BL1) and FSBL (BL2).

BL31 is ATF.
BL33 is the non-secure world software (UEFI, U-Boot, Linux etc).

To build:
```
make CROSS_COMPILE=aarch64-none-elf- PLAT=stratix10
```


