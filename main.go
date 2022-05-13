package main

import (
	"runtime"

	sp "github.com/JoshPattman/spotpuppy-go"
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
	ups := sp.NewUPSTimer(100)
	for {
		r.Update()
		ups.WaitForNext()
	}
}
