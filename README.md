sfhss-study
====
<img alt="elflet" src="https://raw.githubusercontent.com/wiki/opiopan/sfhss-study/images/test.gif" width=350 align="right">

This codes are test imprementation of RF receiver which compliant with Futaba S-FHSS protocol. 
[TI CC2500](http://www.ti.com/product/CC2500/support) is used for RF receiver module,
and it's controlled by [STM32F042K6](https://www.st.com/content/st_com/en/products/microcontrollers-microprocessors/stm32-32-bit-arm-cortex-mcus/stm32-mainstream-mcus/stm32f0-series/stm32f0x2/stm32f042k6.html) microcontroller.<br>
Once data packet sent by Futaba RC transmitter is received, each channel value (up to 8 channels) will be transport to host through USB PHY in STM32F042K6.

This is PoC of [rcstick-f](https://github.com/opiopan/rcstick-f).

## Futaba S-FHSS protocol
S-FHSS is a 2.4 GHz band based RC transceiver protocol designed by Futaba. 
It is not brand new protocol, However many Futaba transmitter suport this protolol. And this protocol is well analyzed. That's why I choose S-FHSS for my own receiver imprementation.

[This commentary article](https://rfengfpv.wordpress.com/2017/01/10/futaba-s-fhss-protocol-overview/) and 
[this C source code](https://github.com/DeviationTX/deviation/blob/2ce0f46fe94d80198ae94fd5a6f6a008863ec420/src/protocol/sfhss_cc2500.c)
which impremens S-FHSS transmitter are very helpful to understand the protocol.
I sincerely appreciate there effort.

## Hardware configuration
Utilizing [this kind of cc2500 module](https://www.aliexpress.com/item/Wireless-Module-CC2500-2-4G-Low-power-Consistency-Stability-Small-Size/32702148262.html?spm=a2g0s.9042311.0.0.27424c4dDEvIUe)
is ease to build a test bench.<br>
This firmware codes subjects to configure STMF042K6 as below.

- SPI1 connects to CC2500 SPI pins
- PA5 connects to CC2500 CSn pin
- PA8 connects to CC2500 GDO0 pin to recognize packet receiving as IRQ
- USB D+ and D- pin is exported.

<p align="center">
<img alt="description" src="https://raw.githubusercontent.com/wiki/opiopan/sfhss-study/images/pocboard.jpg" width=500>
</p>

## Build Firmware
1. **Requirements**<br>
[arm-none-elf-gcc](https://developer.arm.com/open-source/gnu-toolchain/gnu-rm/downloads) 
must be installed in a directory indecateed by `PATH` environment.

2. **Download Source Codes**<br>

    ```shell
    $ git clone https://github.com/opiopan/sfhss-study.git

3. **Compile**
    ```shell
    $ cd sfhss-study
    $ make all