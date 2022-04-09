package main

import (
	"github.com/JoshPattman/spotpuppy-go"
	"runtime"
)

func main() {
	var r *MyRobot
	if (runtime.GOARCH == "amd64"){
		r = NewDummyRobot()
	} else{
		r = NewRobot()
	}
	r.Quad.LoadFromFile("config.json")
	r.Quad.SaveToFile("config.json")
	go updateRobotForever(r)
	startControlApi(r)
}

func updateRobotForever(r *MyRobot){
	ups := spotpuppy.NewUPSTimer(100)
	for true {
		r.Update()
		ups.WaitForNext()
	}
}
