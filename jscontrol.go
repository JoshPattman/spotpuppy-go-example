package main

// File for using ps4 controller over bluetooth to control the robot. You may wish to remap the axis to use a different controller

import (
	"fmt"
	"time"

	"github.com/simulatedsimian/joystick"
)

// The indexes of the different axes returned from simulatedsimian/joystick
const (
	AxisThrottle = 1
	AxisYaw      = 0
	AxisRoll     = 3
	AxisPitch    = 4
)

// Connect to the first joystick that was connected to the computer. Will panic if fails
func ConnectToJS() joystick.Joystick {
	js, err := joystick.Open(0)
	if err != nil {
		panic(err)
	}
	fmt.Printf("Connected to joystick '%s'\n", js.Name())
	return js
}

// This runs the joystick control in the background. It is designed to be run as a goroutine in the background
func RunJSController(js joystick.Joystick, r *Robot) {
	normalizeRange := func(x int) float64 { return -float64(x) / 32767.0 }
	for {
		state, err := js.Read()
		if err != nil {
			panic(err)
		}
		r.VelFwd = normalizeRange(state.AxisData[AxisPitch])
		r.VelLft = normalizeRange(state.AxisData[AxisRoll])
		time.Sleep(time.Second / 10)
	}
}
