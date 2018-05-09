#Tomu Quickstart Guide

This guide describes everything you need to set up your system to develop for [Tomu](https://tomu.im/).

This quickstart guide is designed to be used on Mac, Windows, Linux, and anything else that can run GCC and Make.

## Overview of Requirements

To build and load sample code, you will need three things:

1. An [ARM toolchain](https://developer.arm.com/open-source/gnu-toolchain/gnu-rm)
1. Make
1. dfu-utils

Installation varies depending on your platform:

Platform   | ARM Toolchain  | Make  | dfu-utils
---------- | -------------- | ----- | ----------
*Windows*    | [GNU Arm Embedded Toolchain](https://developer.arm.com/open-source/gnu-toolchain/gnu-rm/downloads) | [GNU Win32 Make](http://gnuwin32.sourceforge.net/packages/make.htm) | [precompiled binaries](http://dfu-util.sourceforge.net/releases/dfu-util-0.8-binaries/win32-mingw32/)
*macOS*      | [GNU Arm Embedded Toolchain](https://developer.arm.com/open-source/gnu-toolchain/gnu-rm/downloads) | [Xcode](https://itunes.apple.com/us/app/xcode/id497799835) | [Homebrew](https://brew.sh/) `brew install dfu-utils`
*Debian/Ubuntu* | `sudo apt-get install gcc-arm-none-eabi` | `sudo apt-get install make` | `sudo apt-get install dfu-util`
*Fedora* | `sudo dnf install arm-none-eabi-newlib  arm-none-eabi-gcc-cs arm-none-eabi-binutils-cs` | `sudo dnf install make` | `sudo dnf install dfu-util`
*Arch* | `sudo pacman -S arm-none-eabi-gcc` | `sudo pacman -S make` | `sudo pacman -S dfu-util`

This quickstart repo differs from the samples repo in that it has a prebuilt version of `libopencm3`, which normally requires various command line programs to compile.  This cuts down on compile time.

## Building Examples

To build an example, go into the directory and type `make`.

## Loading Examples

To load examples onto Tomu, ensure it is in DFU mode by verifying that the red and green LEDs are alternately blinkinking, and that it shows up if you run `dfu-util --list`.  Then, load the sample you want using `dfu-util --download`.