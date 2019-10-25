# HAL2PI
hal to directly access the gpios of a raspberry pi with linuxcnc

This project is based on HAL2Arduino.
See: https://github.com/dewy721/EMC-2-Arduino/tree/master/Downloads/HAL2Arduino

# HAL2PI C install Guide

1.Copy the folder to your LinuxCNC maschine.
2.Install libgpiod. See https://forum.armbian.com/topic/6249-build-libgpiod-the-new-gpio-interface-for-user-space/
3.Go to linuxcnc-dev/scripts
4.Execute: . ./rip-environment
5.Modify linuxcnc-dev/src/Makefile.modinc in line 86 with LDFLAGS = -lgpiod or copy the given file
6.Go back to other machine folder
7.Execute: halcompile --install HAL2PI.c 
8.Have fun

The Example uses pin 18 at output of enable joint0 and pin 17 as input but no destination 

To change the inputs and outputs, change loadrt HAL2PI cfg="in 17 out 18" in .hal

Note: this project is based on hal_parport. There is currently not the same reset function that is given there.

# Compiling libgpiod 
https://forum.armbian.com/topic/6249-build-libgpiod-the-new-gpio-interface-for-user-space/

    sudo armbian-config, Software, Headers
    sudo apt-get install libtool pkg-config
    git clone https://git.kernel.org/pub/scm/libs/libgpiod/libgpiod.git
    cd libgpiod
    mkdir -p include/linux
    cp /usr/src/linux-headers-$(uname -r)/include/linux/compiler_types.h include/linux/.
    ./autogen.sh --enable-tools=yes --prefix=/usr/local CFLAGS="-I/usr/src/linux-headers-$(uname -r)/include/uapi -Iinclude"
    make
    sudo make install
    sudo ldconfig

Let's try some commands:
sudo gpiodetect 



NOTES:
This software is given at it is. I give to warranty of success. It is not recommended to use this software for productionmaschines.
