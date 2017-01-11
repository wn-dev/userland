This repository contains the source code for the ARM side libraries used on Raspberry Pi.
These typically are installed in /opt/vc/lib and includes source for the ARM side code to interface to:
EGL, mmal, GLESv2, vcos, openmaxil, vchiq_arm, bcm_host, WFC, OpenVG.

Use buildme to build. It requires cmake to be installed and an arm cross compiler. It is set up to use this one:
https://github.com/raspberrypi/tools/tree/master/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian

This is a fork used to build raspimjpeg as used in RPi_Web_Cam

I recommend building with gcc-4.6. I have experienced some PIPE reading issues when building with later versions.

To do this 
sudo apt-get install gcc-4.6 g++-4.6

and then run
chmdod u+x gcc-version.sh
./gcc-version.sh 4.6
