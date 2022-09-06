package main

import (
	"os"
	"runtime"

	sp "github.com/JoshPattman/spotpuppy-go"
)

func main() {
	var r *Robot
	if runtime.GOARCH == "amd64" {
		r = NewRobot(sp.NewDummyMotorController(), sp.NewDummyRotationSensor())
	} else {
		r = NewRobot(
			sp.NewPCAMotorController(),
			sp.NewArduinoRotationSensor("/dev/ttyUSB0", sp.AxesRemap{X: "x", Y: "y", Z: "z"}),
		)
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
