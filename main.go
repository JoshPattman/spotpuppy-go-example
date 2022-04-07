package main

import (
	"fmt"
	"github.com/JoshPattman/spotpuppy-go"
	"time"
)

func main() {
	r := NewRobot()
	r.Quad.LoadFromFile("config.json")
	r.Quad.SaveToFile("config.json")
	t := time.Now()
	ups := spotpuppy.NewUPSTimer(100)
	steps := 100.0
	for i := 0.0; i < steps; i++ {
		r.Update()
		ups.WaitForNext()
	}
	s := time.Since(t)
	fmt.Println(int(steps/s.Seconds()), "ups")
	fmt.Print("(", s/time.Duration(steps), " per update)")
}
