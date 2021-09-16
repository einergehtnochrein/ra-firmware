# Ra Radiosonde Receiver - Firmware

This is the firmware for the Ra Radiosonde Receiver.
For a project overview see [Ra Hardware](https://github.com/einergehtnochrein/ra-hardware)

## Compiling the Code

Code development is done an a Debian Linux machine with the arm-none-eabi variant of the GCC compiler. Version 2017-q4 of the compiler was used, but any later version should do as well (although not tested). You can get the compiler from the [ARM website](https://developer.arm.com/open-source/gnu-toolchain/gnu-rm/downloads).

Assuming the GCC compiler is in your path, go to the root directory and type

        make
You can use 

        make clean
to remove all previous compiler output.

If all goes well, the output is:

        gcc/obj_m4_ra2/ra2_firmware.axf
        gcc/obj_m4_ra2/ra2_firmware.hex

The ELF file (`.axf`) is for use with the debugger, while the `.hex` file is to be used with the Android app for field updates.

## Installation

In a development environment, you are expected to use a debugger which connects to the SWD port of the LPC54000 microcontroller of the receiver. There's a plethora of hardware and software available for this purpose, however, I personally prefer the [Lauterbach TRACE32](http://lauterbach.com). It's the best.

End users can do a field upgrade of the receiver firmware through the Android app. All that's required is the `.hex` file produced by the compiler.

## License

This project is licensed under the BSD License - see the [LICENSE](LICENSE) file for details

