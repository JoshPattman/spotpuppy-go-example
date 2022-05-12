package main

import (
	"fmt"
	"runtime"

	"github.com/JoshPattman/spotpuppy-go"
)

func PrintVec(v *spotpuppy.Vec3) {
	fmt.Println("(", v.X, ",", v.Z, ",", v.Y, ")")
}

func main() {
	a := spotpuppy.NewQuatAngleAxis(spotpuppy.DirLeft, 45)
	b := a.RotateByGlobal(spotpuppy.NewQuatAngleAxis(spotpuppy.DirUp, 90))
	fmt.Println(a.Rotate(spotpuppy.DirForward))
	fmt.Println(b.Rotate(spotpuppy.DirForward))
	_, y, _ := b.Euler()
	c := spotpuppy.NewQuatAngleAxis(spotpuppy.DirUp, y*180.0/3.14)
	fmt.Println(b.RotateByLocal(c).Rotate(spotpuppy.DirForward))
}

func main2() {
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
