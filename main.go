package main

import (
	"flag"
	"fmt"
	"os"
	"runtime"

	sp "github.com/JoshPattman/spotpuppy-go"
	"github.com/simulatedsimian/joystick"
)

var mode string

func main() {
	// Parse cli
	flag.StringVar(&mode, "m", "stand", "the operation for the robot to do (one of: 'trot-in-place', 'trot-forwards', 'stand', 'stand-tall', 'stand-front-left', 'balance', 'trot-js')")
	flag.Parse()

	// create robot with appropriate peripherals
	fmt.Println("Creating robot")
	var r *Robot
	if runtime.GOARCH == "amd64" {
		r = NewRobot(sp.NewDummyMotorController(), sp.NewDummyRotationSensor())
		fmt.Println("Created dummy robot (as we are not on a rpi)")
	} else {
		r = NewRobot(
			sp.NewPCAMotorController(),
			sp.NewArduinoRotationSensor("/dev/ttyUSB0", sp.AxesRemap{X: "x", Y: "y", Z: "z"}),
		)
		fmt.Println("Created PCA and arduino robot")
	}

	// setup the robots mode
	useJoystick := false
	switch mode {
	case "trot-in-place":
		r.Mode = ModeTrot
	case "trot-forwards":
		r.Mode = ModeTrot
		r.VelFwd = 3
	case "stand":
		r.Mode = ModeStand
	case "stand-tall":
		r.Mode = ModeStandTall
	case "stand-front-left":
		r.Mode = ModeStandFL
	case "balance":
		r.Mode = ModeBalance
	case "trot-js":
		r.Mode = ModeTrot
		useJoystick = true
	default:
		panic("Mode not recognised")
	}
	fmt.Println("Starting robot with mode", mode)

	var js joystick.Joystick
	if useJoystick {
		fmt.Println("Connecting to joystick")
		js = ConnectToJS()
	}

	// load the quadruped settings from file
	fmt.Println("Loading config from disk")
	if _, err := os.Stat("quad.json"); err != nil {
		if err = r.Quadruped.SaveToFile("quad.json"); err != nil {
			panic(err)
		}
	}
	if err := r.Quadruped.LoadFromFile("quad.json"); err != nil {
		panic(err)
	}

	// begin updating the robot velocities in the background
	if useJoystick {
		go RunJSController(js, r)
	}
	// update the robot at 100 times per second
	fmt.Println("Updating robot")
	ups := sp.NewUPSTimer(100)
	for {
		r.Update()
		ups.WaitForNext()
	}
}
