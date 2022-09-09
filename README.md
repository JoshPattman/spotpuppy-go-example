# Example Usage of `spotpuppy-go`
> This is an example usage of the [spotpuppy-go](https://github.com/JoshPattman/spotpuppy-go) module
## Robot Hardware
* RPi4 running headless raspberry pi os
* Arduino nano connected over usb cable to RPi, running [this](https://github.com/JoshPattman/arduino-raw-mpu5060) sketch
	* Soldered onto this is an `MPU6050` mpu. Instructions can be found in the sketch.
* `pca9685` servo control hat for RPi
* 12 `MG90D` servos plugged into the servo control hat
* Amazon 12V-5V voltage regulator
* 3S LiPo battery
* 3D printed robot ([repo](https://github.com/JoshPattman/spotpuppy-models))
## Files/Folders
### `conf-dummy`
This contains the configuration for the robot, such as joint offsets, which ports each servo is plugged into, and tuning values for the gait. You will probably have to modify this for your robot. Use this as a template for your own robot.
### `sp5-conf`
This contains the configuration for my robot I am currently working on. It is the same structure as `conf-dummy` but I have entered the calibration params.
### `robot.go`
This contains the walking algorithm and actual usage of the `spotpuppy-go` module. It also contains all of the code for making the robot stand and balance.
### `main.go`
This is the entrypoint to the program. On RPi it will create a robot with a functional movement controller and rotation sensor, however on other platforms it will use dummy controller for these (they don't work on windows/x64 linux). It also contains the code to make the robot update at a fixed timestep of 100 times per second. For instructions of how to run it, type `go run . -h` when inside of this directory
### `gait.go`
This contains code for generating foot position offsets from the floor to make the robot trot. It is not concerned with the rotation/coordinate systems of the robot, but instead is simply a couple of functions that take some parameters for a step and a time, and return position offsets.
## `jscontrol.go`
This contains code for communicating with a ps4 controller connected over bluetooth and controlling the speed of the robot with the right joystick
## `pid.go`
This contains a very simple PI controller. I have intentionally left out the D term for now, however in future I may add it back in.