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
		Quad:   sp.NewQuadruped(sp.NewDirectMotorIK(), sp.NewDummyMotorController()),
		Sensor: &sp.DummyRotationSensor{},
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
	stepA := r.CS.TransformDirection(sp.NewVector3(0, -snA, 0))
	stepB := r.CS.TransformDirection(sp.NewVector3(0, -snB, 0))
	flPos := r.Quad.GetVectorToShoulder(sp.LegFrontLeft).Add(r.CS.TransformDirection(sp.NewVector3(0, restHeight, 0))).Add(r.CS.TransformDirection(r.Quad.GetVectorToShoulder(sp.LegFrontLeft).Inv()))
	frPos := r.Quad.GetVectorToShoulder(sp.LegFrontRight).Add(r.CS.TransformDirection(sp.NewVector3(0, restHeight, 0))).Add(r.CS.TransformDirection(r.Quad.GetVectorToShoulder(sp.LegFrontRight).Inv()))
	blPos := r.Quad.GetVectorToShoulder(sp.LegBackLeft).Add(r.CS.TransformDirection(sp.NewVector3(0, restHeight, 0))).Add(r.CS.TransformDirection(r.Quad.GetVectorToShoulder(sp.LegBackLeft).Inv()))
	brPos := r.Quad.GetVectorToShoulder(sp.LegBackRight).Add(r.CS.TransformDirection(sp.NewVector3(0, restHeight, 0))).Add(r.CS.TransformDirection(r.Quad.GetVectorToShoulder(sp.LegBackRight).Inv()))
	r.Quad.SetLegPosition(sp.LegFrontLeft, flPos.Add(stepA))
	r.Quad.SetLegPosition(sp.LegFrontRight, frPos.Add(stepB))
	r.Quad.SetLegPosition(sp.LegBackLeft, blPos.Add(stepB))
	r.Quad.SetLegPosition(sp.LegBackRight, brPos.Add(stepA))
	// Update quad
	r.Quad.Update()
}
