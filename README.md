# CRW 2021-2022 Payload Repo
This repository houses the software for the Charger Rocket Works (CRW) payload. The payload this year is a navigation device that will determine the rockets landing spot without GNSS signals. The team has opted for to use Dead Reckoning. The software includes several versions:

- Workspace: Compiled C++ executable for running on a laptop. Allows for playing back old data (not currently implemented)
- Teensy Build: Build that will run on a teensy 4.1
- beagle bone build: Runs on a beagle bone

### Clone the Repository

The repository can be cloned via:

```git clone https://github.com/UAHCRW/Payload.git [folder name (optional)]```

Currently the repository is setup to work the best with vsCode which can be downloaded at the following link. https://code.visualstudio.com/

### Build Teensy 4.1

These instructions are for building and programming the Teensy 4.1. There are two pieces of software required for building code for the teensy. 

- Arduino -> https://www.arduino.cc/en/software
- Teensyduino -> https://www.pjrc.com/teensy/td_download.html
- Visual Teensy -> https://github.com/luni64/VisualTeensy/releases/tag/v1.4.0

##### VsCode Instructions

1. ```ctrl+shift+b``` and choose ```Build``` . This will build the code but not upload it
2. ```ctrl+shift_b``` and choose ```Upload (TyCommander)```. This will upload the code to the Teensy
3. ```ctrl+shift+b``` and choose ```Clean``` . This will delete all built files to do a fresh build

Optionally, the program may be built for the Teensy 4.0. To do this change the name of the ```makefile``` to ```makefile41``` and then change the name of ```makefile40``` to ```make```. To swap back to the 4.1 build just do the reverse.

### Build Beagle Bone (Coming soon to a Beagle Bone near you)



### Build Workspace (Coming soon to your laptop)





