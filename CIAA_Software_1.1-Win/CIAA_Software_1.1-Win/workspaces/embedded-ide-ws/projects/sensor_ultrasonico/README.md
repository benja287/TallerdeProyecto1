## Available libraries:

- LPCOpen v3.01
- sAPI r0.5.0.
- FreeRTOS Kernel V10.0.1
- Elm-Chan FatFS. Support for SD Cards (SSP) & Pendrives (USB MSD).

## Supported boards

- CIAA-NXP (LPC4337).
- EDU-CIAA-NXP (LPC4337).

## Supported toolchains

- gcc-arm-none-eabi

## Usage

Make sure you have an ```arm-none-eabi-*``` toolchain configured in your ```PATH```. If you don't have it, download [GCC ARM Embedded](https://developer.arm.com/open-source/gnu-toolchain/gnu-rm).
- Define ```PROJECT_NAME```  and ```PROJECT_PATH``` variables in ```project.mk``` according to the project you want to compile (PROJECT_PATH is relative to this folder, leave void if the project is in this folder).
- Compile with ```make```.
- Clean with ```make clean```.

Make sure you have an ```openocd``` configured in your ```PATH```.
- Download to target via OpenOCD with ```make download```.

## Add a new library

The ```libs``` folder allow you to include 2 types of libraries:
- Simplie library. Consist in a folder (with a non-spaces name) that includes inside 2 folders, one named ```src``` (here go .c, .cpp or .s source code files), and another one named ```inc``` (here go .h or .hpp header files). This kind of library compiles automaticaly by the Makefile.
- Advanced library. Consist in a library whit a complex folder and files strcuture, i.e. LibUSB. This case reuire make your own makefile. You can inspire from sAPI makefile to do that.

## Examples

See more examples at: https://github.com/epernia/cese-edu-ciaa-template
