Github Repo for the lightning detector, AKA the THUNDER BUDDY.

All source code downloads require installation of the pico-sdk, instructions can be found at https://www.raspberrypi.com/news/get-started-with-raspberry-pi-pico-series-and-vs-code/
It is recommended to install in VSCode as the firmware developers developed, tested, and integrated in the VSCode environment

For Linux terminal use:
Clone this repo\
cd ECE411-THUNDER-BUDDIES/software/src\
mkdir build\
cd build\
cmake ..\
make ThunderBuddy\
mv ThunderBuddy.elf pico/dir/here

CMake options:\
-DPRINT will print messages to stdout\
-DFLASH will flash the onboard LED when transmitting to 7020\
-DFORCE will force an lightning detected condition always\
-DPRINTFLASH combines flash and print\
-DPRINTFORCE combines force and print\
-DDEBUG enables all debug modes\
