# NuttX Flight Computer
This is the repository for a Flight Computer configured to run on the NuttX RTOS
1. This project is a work in progress
2. This project is being developed by the WPI High Power Rocketry Team
3. This project is developed for the buildhost to be using either a Linux or MacOS operating system
    * The NuttX website has further information on how to configure the build environment for Windows

## Build Instructions
### Prerequisites
1. Install the NuttX toolchain\
To install the NuttX toolchain, please follow the instructions at the following link: https://nuttx.apache.org/docs/latest/quickstart/install.html
** No need to download NuttX, it is already here **

2. Configure the Pico SDK\
To configure the Pico SDK, please pull the git submodules, and a pico-sdk folder should appear. Then, run the following commands:
```bash
# If using bash terminal environment, use the following
echo 'export PICO_SDK_PATH=$PWD' >> ~/.bashrc
# If using zsh terminal environment, use the following
echo 'export PICO_SDK_PATH=$PWD' >> ~/.zshrc

source ~/.bashrc 
# OR
source ~/.zshrc
make
```

### Configure Flight Computer NuttX Environment
1. Configure the build environment for the flight computer:
```bash
cd nuttx
# If using linux
./tools/configure.sh -l ./CustomBoards/HPRC_FC/configs/debug/

# If using MacOS
./tools/configre.sh -m ./CustomBoards/HPRC_FC/configs/debug/
```
2. Build Flight Computer
```
sudo make
```
At this time, there is no way to DFU upload, so the binary must be manually uploaded to the flight computer. \
If the RP2040 environment is configured properly, there will be a nuttx.uf2 file in the directory.