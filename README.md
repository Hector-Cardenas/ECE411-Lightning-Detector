Github Repo for the lightning detector, AKA the THUNDER BUDDY.

All source code downloads require installation of the pico-sdk, instructions can be found at https://www.raspberrypi.com/news/get-started-with-raspberry-pi-pico-series-and-vs-code/ \
Building and compiling was done on a linux machine to produce a .uf2 file to drag and drop into the pico w when in BOOTSEL mode. This is the recommended way to build the project\
For Linux terminal use:\
Clone this repo\
cd ECE411-THUNDER-BUDDIES/software/src\
mkdir build\
cd build\
cmake ..\
make ThunderBuddy\
Drag and drop the .uf2 file into the pico w 

Re-making the project just requires a Make clean and a Make Thunderbuddy
If any of the debug options want to be added or removed one must remove the CMakeCache.txt file in /build before using a new cmake command


CMake options (set the option to =1 to select the argument):\
-DPRINT will print messages to stdout\
-DFLASH will flash the onboard LED when transmitting\
-DFORCE will force an lightning detected condition always\
-DPRINTFLASH combines flash and print\
-DPRINTFORCE combines force and print\
-DDEBUG enables all debug modes\
