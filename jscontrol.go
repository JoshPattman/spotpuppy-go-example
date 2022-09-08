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

	ButtonX = 0
	ButtonO = 1
	ButtonT = 2
	ButtonS = 3
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

func pressed(buttons uint32, button int) bool {
	return (buttons>>button)&0x01 == 1
}
func pressedDown(buttons, lastbuttons uint32, button int) bool {
	return (!pressed(lastbuttons, button)) && (pressed(buttons, button))
}

// This runs the joystick control in the background. It is designed to be run as a goroutine in the background
func RunJSController(js joystick.Joystick, r *Robot) {
	normalizeRange := func(x int) float64 { return -float64(x) / 32767.0 }
	var lastbuttons uint32
	for {
		// Read current state
		state, err := js.Read()
		if err != nil {
			panic(err)
		}
		// Do stuff with inputs
		r.VelFwd = normalizeRange(state.AxisData[AxisPitch])
		r.VelLft = normalizeRange(state.AxisData[AxisRoll])
		if r.Mode != ModeStand && pressedDown(state.Buttons, lastbuttons, ButtonX) {
			r.Mode = ModeStand
		}

		if r.Mode != ModeTrot && pressedDown(state.Buttons, lastbuttons, ButtonS) {
			r.Mode = ModeTrot
		}

		// Wait for next update
		lastbuttons = state.Buttons
		time.Sleep(time.Second / 10)
	}
}
