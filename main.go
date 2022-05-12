package main

import (
	"runtime"

	"github.com/JoshPattman/spotpuppy-go"
)

func main() {
	var r *MyRobot
	if runtime.GOARCH == "amd64" {
		r = NewDummyRobot()
	} else {
		r = NewRobot()
	}
	r.Load("conf")
	r.Save("conf")
	go updateRobotForever(r)
	startControlApi(r)
}

func updateRobotForever(r *MyRobot) {
	ups := spotpuppy.NewUPSTimer(100)
	for true {
		r.Update()
		ups.WaitForNext()
	}
}
