# jellyfish
Code for capturing position of BLOOPER via Arduino UNO using April tags and a standard camera. 

## Project Description

This repository contains all the scripts necessary for running BLOOPER (backronym tbd) for hardware validation of Aquarium. There are 3 main parts of this project:

1. A camera calibration folder that contains the scripts necessary for getting camera parameters. Assumes camera is connected via USB to the host computer. Camera scripts originally taken from https://github.com/partomatl/drone-tracking.

2. Python scripts that get the current position of the Aruco marker on BLOOPER and send commands to the Arduino accordingly.

2. A arduino script (jellyfish.ino) that receives angle commands from the python scripts and controls the servos on BLOOPER

## Dependencies

Arduino 2.1+ and the VSCode extension for Arduino are required. 