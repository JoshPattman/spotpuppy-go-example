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
	b := spotpuppy.NewQuatAngleAxis(spotpuppy.DirForward, 30)
	c := spotpuppy.NewQuatAngleAxis(spotpuppy.DirUp, 45)
	ab := a.RotateByLocal(b)
	abc := ab.RotateByGlobal(c)
	fmt.Println(ab.Apply(spotpuppy.DirForward))
	fmt.Println(abc.Apply(spotpuppy.DirForward))
	fmt.Println(abc.NoYaw().Apply(spotpuppy.DirForward))
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
