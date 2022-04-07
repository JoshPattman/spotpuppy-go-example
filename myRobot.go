package main

import (
	sp "github.com/JoshPattman/spotpuppy-go"
	"math"
	"time"
)

type MyRobot struct {
	Quad   *sp.Quadruped
	Sensor sp.RotationSensor
	CS     *sp.RollPitchCoordinateSystem
	T      float64
	LT     time.Time
}

func NewRobot() *MyRobot {
	return &MyRobot{
		Quad:   sp.NewQuadrupedWithExtraMotors(sp.NewDirectMotorIKGenerator(), sp.NewDummyMotorController(), []string{"neck"}),
		Sensor: sp.NewDummyRotationSensor(),
		CS:     sp.NewRollPitchCoordinateSystem(),
		T:      0,
		LT:     time.Now(),
	}
}

func (r *MyRobot) Update() {
	// Timekeeping
	r.T += time.Since(r.LT).Seconds()
	r.LT = time.Now()
	clkA := math.Mod(r.T, 1.0)
	clkB := math.Mod(r.T+0.5, 1.0)
	// Update rotation sensor
	r.CS.UpdateRollPitchFrom(r.Sensor)
	// Custom walking code
	snA := math.Sin(clkA * 3.1415 * 2)
	snB := math.Sin(clkB * 3.1415 * 2)
	snA = math.Min(snA, 0)
	snB = math.Min(snB, 0)
	restHeight := r.Quad.Legs[sp.LegFrontLeft].GetRestingPosition().Y
	stepA := r.CS.TD(sp.DirUp.Mul(snA))
	stepB := r.CS.TD(sp.DirUp.Mul(snB))
	straightDown := r.CS.TD(sp.DirDown.Mul(restHeight))
	for _, l := range sp.AllLegs {
		floorPos := r.Quad.ShoulderVec(l).Add(straightDown).Add(r.CS.TD(r.Quad.ShoulderVec(l).Inv()))
		var step *sp.Vector3
		if l == sp.LegFrontLeft || l == sp.LegBackRight {
			step = stepA
		} else {
			step = stepB
		}
		r.Quad.SetLegPosition(l, floorPos.Add(step))
	}
	// Update quad
	r.Quad.Update()
}
