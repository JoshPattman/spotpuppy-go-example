package main

import (
	"flag"
	"fmt"
	"os"
	"runtime"

	sp "github.com/JoshPattman/spotpuppy-go"
)

var mode string

func main() {
	// Parse cli
	flag.StringVar(&mode, "m", "stand", "the operation for the robot to do (one of: 'trot-in-place', 'trot-forwards', 'stand', 'stand-tall', 'stand-front-left', 'balance')")
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
	switch mode {
	case "trot-in-place":
		r.Mode = ModeTrot
	case "trot-forwards":
		panic("Mode not implemented")
	case "stand":
		r.Mode = ModeStand
	case "stand-tall":
		r.Mode = ModeStandTall
	case "stand-front-left":
		r.Mode = ModeStandFL
	case "balance":
		r.Mode = ModeBalance
	default:
		panic("Mode not recognised")
	}
	fmt.Println("Starting robot with mode", mode)

	// load the quadruped settings from file
	fmt.Println("Loading config from disk")
	if _, err := os.Stat("quad.json"); err != nil {
		r.Quadruped.SaveToFile("quad.json")
	}
	r.Quadruped.LoadFromFile("quad.json")

	// update the robot at 100 times per second
	fmt.Println("Updating robot")
	ups := sp.NewUPSTimer(100)
	for {
		r.Update()
		ups.WaitForNext()
	}
}
