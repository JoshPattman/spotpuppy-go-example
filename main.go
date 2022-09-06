package main

import (
	"flag"
	"os"
	"runtime"

	sp "github.com/JoshPattman/spotpuppy-go"
)

var mode string

func main() {
	flag.StringVar(&mode, "m", "trot-in-place", "the operation for the robot to do (one of: 'trot-in-place', 'trot-forwards', 'stand', 'stand-tall', 'stand-front-left', 'balance')")
	flag.Parse()
	var r *Robot
	if runtime.GOARCH == "amd64" {
		r = NewRobot(sp.NewDummyMotorController(), sp.NewDummyRotationSensor())
	} else {
		r = NewRobot(
			sp.NewPCAMotorController(),
			sp.NewArduinoRotationSensor("/dev/ttyUSB0", sp.AxesRemap{X: "x", Y: "y", Z: "z"}),
		)
	}
	switch mode {
	case "trot-in-place":
	case "trot-forwards":
		panic("Mode not implemented")
	case "stand":
		panic("Mode not implemented")
	case "stand-tall":
		panic("Mode not implemented")
	case "stand-front-left":
		panic("Mode not implemented")
	case "balance":
		panic("Mode not implemented")
	default:
		panic("Mode not recognised")
	}
	if _, err := os.Stat("quad.json"); err != nil {
		r.Quadruped.SaveToFile("quad.json")
	}
	r.Quadruped.LoadFromFile("quad.json")

	ups := sp.NewUPSTimer(100)
	for {
		r.Update()
		ups.WaitForNext()
	}
}
