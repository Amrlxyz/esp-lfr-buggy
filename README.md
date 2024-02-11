# Bare Metal Blinky

This guide reviews the steps required to get Blinky with Bare Metal working on an Mbed OS platform.

Please install [mbed CLI](https://github.com/ARMmbed/mbed-cli#installing-mbed-cli).

## Import the example application

From the command-line, import the example:

```
mbed import mbed-os-example-blinky-baremetal
cd mbed-os-example-blinky-baremetal
```

### Now compile

Invoke `mbed compile`, and specify the name of your platform and your favorite toolchain (`GCC_ARM`, `ARM`, `IAR`). For example, for the ARM Compiler:

```
mbed compile -m K64F -t ARM
```

Your PC may take a few minutes to compile your code. At the end, you see the following result:

```
[snip]
Image: ./BUILD/K64F/GCC_ARM/mbed-os-example-blinky-baremetal.bin
```

### Program your board

1. Connect your mbed device to the computer over USB.
1. Copy the binary file to the mbed device.
1. Press the reset button to start the program.

The LED on your platform turns on and off.

## Troubleshooting

If you have problems, you can review the [documentation](https://os.mbed.com/docs/latest/tutorials/debugging.html) for suggestions on what could be wrong and how to fix it.

## Related Links

* [Mbed OS Configuration](https://os.mbed.com/docs/latest/reference/configuration.html)

### License and contributions

The software is provided under Apache-2.0 license. Contributions to this project are accepted under the same license. Please see contributing.md for more info.

This project contains code from other projects. The original license text is included in those source files. They must comply with our license guide.
