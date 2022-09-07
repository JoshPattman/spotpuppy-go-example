# Example Usage of `spotpuppy-go`
> This is an example usage of the [spotpuppy-go](https://github.com/JoshPattman/spotpuppy-go) module
## Robot Hardware
* RPi4 running headless raspberry pi os
* Arduino nano connected over usb cable to RPi, running the `simple` branch of [this](https://github.com/JoshPattman/arduino-mpu6050) sketch
	* Soldered onto this is an `MPU6050` dmp. Instructions can be found in the sketch.
* `pca9685` servo control hat for RPi
* 12 `MG90D` servos plugged into the servo control hat
* Amazon 12V-5V voltage regulator
* 3S LiPo battery
* 3D printed robot ([repo](https://github.com/JoshPattman/spotpuppy-models))
## Files/Folders
### `conf-dummy`
This contains the configuration for the robot, such as joint offsets, which ports each servo is plugged into, and tuning values for the gait. You will probably have to modify this for your robot.
### `robot.go`
This contains the walking algorithm and actual usage of the `spotpuppy-go` module. It also contains all of the code for making the robot stand and balance.
### `main.go`
This is the main file that runs the rest of the code. On RPi it will create a robot with a functional movement controller and rotation sensor, however on other platforms it will use dummy controller for these (they don't work on windows/x64 linux). It also contains the code to make the robot update at a fixed timestep of 100 times per second
### `gait.go`
This contains code for generating foot position offsets from the floor to make the robot trot. It is not concerned with the rotation/coordinate systems of the robot, but instead is simply a couple of functions that take some parameters for a step and a time, and return position offsets.