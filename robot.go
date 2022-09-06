package main

// This is the new robot logic. I have not finished writing it yet

import (
	"math"
	"time"

	sp "github.com/JoshPattman/spotpuppy-go"
)

type Robot struct {
	Quadruped      *sp.Quadruped
	RotationSensor sp.RotationSensor
	t              float64
	lastUpdate     time.Time
	GaitParameters *GaitParameters
	StepFrequency  float64
	BodyHeight     float64
}

func NewRobot(motorController sp.MotorController, rotationSensor sp.RotationSensor) *Robot {
	q := sp.NewQuadruped(sp.NewDirectMotorIKGenerator(), motorController)
	return &Robot{
		Quadruped:      q,
		RotationSensor: rotationSensor,
		lastUpdate:     time.Now(),
		GaitParameters: &GaitParameters{
			Airtime:          0.33,
			StepHeight:       2,
			ExtendHorizontal: 0.4,
		},
		StepFrequency: 2,
		BodyHeight:    8,
	}
}

func (r *Robot) Update() {
	// Timekeeping
	r.t += time.Since(r.lastUpdate).Seconds() * r.StepFrequency
	r.lastUpdate = time.Now()

	// Rotation
	bodyRotation := r.RotationSensor.GetQuaternion().NoYaw()
	//hasFallen := sp.DirUp.AngleTo(sp.DirUp.Rotated(bodyRotation)) > 30
	r.updateTrot(bodyRotation)
}

// For now, this makes no attempt to right itself, instead will just slowly fall over
func (r *Robot) updateTrot(bodyRotation sp.Quat) {
	// Clocks
	clkA := math.Mod(r.t, 1.0)
	clkB := math.Mod(r.t+0.5, 1.0)
	clks := make(map[string]float64)
	clks[sp.LegFrontLeft], clks[sp.LegBackRight] = clkA, clkA
	clks[sp.LegFrontRight], clks[sp.LegBackLeft] = clkB, clkB

	// Dynamic params
	stepLengthsFwd, stepLengthsLft := make(map[string]float64), make(map[string]float64)
	pushForces := make(map[string]float64)
	for _, l := range sp.AllLegs {
		stepLengthsFwd[l], stepLengthsLft[l], pushForces[l] = 0, 0, 0
	}

	// TODO : Here is where we need to write code to determine push forces and step lengths

	// Convert to gait
	globalDown := sp.DirDown.Mul(r.BodyHeight).Rotated(bodyRotation)
	for _, l := range sp.AllLegs {
		// Get the gait offset
		up := r.GaitParameters.VerticalOffsetFromFloor(pushForces[l], clks[l])
		fwd := r.GaitParameters.HorizontalOffset(stepLengthsFwd[l], clks[l])
		lft := r.GaitParameters.HorizontalOffset(stepLengthsLft[l], clks[l])
		// This offset is in local rotation space. We want to rotate it to global space
		offsetRelative := sp.DirUp.Mul(up).Add(sp.DirForward.Mul(fwd)).Add(sp.DirLeft.Mul(lft))
		offset := offsetRelative.Rotated(bodyRotation)

		// Get the floor pos
		floorPos := r.Quadruped.ShoulderVec(l).Inv().Add(globalDown).Add(r.Quadruped.ShoulderVec(l)).Add(r.Quadruped.ShoulderVec(l).Rotated(bodyRotation))

		// Set foot pos
		r.Quadruped.SetLegPosition(l, floorPos.Add(offset))
	}
}
