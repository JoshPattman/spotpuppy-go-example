package main

// This is the new robot logic. I have not finished writing it yet

import (
	"math"
	"time"

	sp "github.com/JoshPattman/spotpuppy-go"
)

type RobotMode int

const (
	ModeStand RobotMode = iota
	ModeStandTall
	ModeStandFL
	ModeBalance
	ModeTrot
)

type TrotParameters struct {
	GaitParameters *GaitParameters `json:"gait-gen-params"`
	StepFrequency  float64         `json:"step-freq"`
	BodyHeight     float64         `json:"body-height"`
	PushPIFwd      *PIController   `json:"push-force-pi-fwd"`
	PushPILft      *PIController   `json:"push-force-pi-lft"`
}

type Robot struct {
	Quadruped      *sp.Quadruped
	RotationSensor sp.RotationSensor
	t              float64
	lastUpdate     time.Time
	TrotParameters *TrotParameters
	Mode           RobotMode
	VelFwd, VelLft float64
}

func NewRobot(motorController sp.MotorController, rotationSensor sp.RotationSensor) *Robot {
	q := sp.NewQuadruped(sp.NewDirectMotorIKGenerator(), motorController)
	return &Robot{
		Quadruped:      q,
		RotationSensor: rotationSensor,
		lastUpdate:     time.Now(),
		TrotParameters: &TrotParameters{
			GaitParameters: &GaitParameters{},
			PushPIFwd:      NewPIController(0, 0),
			PushPILft:      NewPIController(0, 0),
		},
		Mode: ModeStand,
	}
}

func (r *Robot) Update() {
	// Timekeeping
	r.t += time.Since(r.lastUpdate).Seconds() * r.TrotParameters.StepFrequency
	r.lastUpdate = time.Now()

	// Rotation
	bodyRotation := r.RotationSensor.GetQuaternion().NoYaw()
	//hasFallen := sp.DirUp.AngleTo(sp.DirUp.Rotated(bodyRotation)) > 30
	switch r.Mode {
	case ModeStand:
		r.updateStand(sp.NewVector3(0, 0, 0))
	case ModeStandTall:
		r.updateStand(sp.DirDown.Mul(3))
	case ModeStandFL:
		r.updateStand(sp.DirForward.Mul(3).Add(sp.DirLeft.Mul(3)))
	case ModeBalance:
		r.updateBalance(bodyRotation)
	case ModeTrot:
		r.updateTrot(bodyRotation)
	}
}

// The basic idea of this tro algo is as follows:
// -> Each leg has push force proportional to how far the robot is tilting towards it
// -> Generate floor offsets from gait.go file with the push forces descibed above. Each diagonal leg pair has the same timing for its gait (both pairs are offset by half a cycle from each other)
// -> Calculate the position on the floor directly below each shoulder (this takes into account the body rotation)
// -> Add the floor offsets to this position on the floor. Each leg should now be trotting
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
	{
		// Calculate rotated upwards vector
		rotatedUp := sp.DirUp.Rotated(bodyRotation)

		// Calculate PID values from tilting forwards and left angles
		// Positive means tilting forwards
		r.TrotParameters.PushPIFwd.Current = math.Asin(sp.DirForward.Dot(rotatedUp))
		// Positive means tilting left
		r.TrotParameters.PushPILft.Current = math.Asin(sp.DirLeft.Dot(rotatedUp))
		pushFwd := r.TrotParameters.PushPIFwd.NextAdjustment()
		pushLft := r.TrotParameters.PushPILft.NextAdjustment()

		// Set the push forces of each leg
		pushForces[sp.LegFrontLeft] = pushFwd + pushLft
		pushForces[sp.LegFrontRight] = pushFwd - pushLft
		pushForces[sp.LegBackLeft] = -pushFwd + pushLft
		pushForces[sp.LegBackRight] = -pushFwd - pushLft

		for _, l := range sp.AllLegs {
			stepLengthsFwd[l] = r.VelFwd / r.TrotParameters.StepFrequency
			stepLengthsLft[l] = r.VelLft / r.TrotParameters.StepFrequency
		}
	}

	// Convert to gait
	globalDown := sp.DirDown.Mul(r.TrotParameters.BodyHeight).Rotated(bodyRotation)
	for _, l := range sp.AllLegs {
		// Get the gait offset
		up := r.TrotParameters.GaitParameters.VerticalOffsetFromFloor(pushForces[l], clks[l])
		fwd := r.TrotParameters.GaitParameters.HorizontalOffset(stepLengthsFwd[l], clks[l])
		lft := r.TrotParameters.GaitParameters.HorizontalOffset(stepLengthsLft[l], clks[l])
		// This offset is in local rotation space. We want to rotate it to global space
		offsetRelative := sp.DirUp.Mul(up).Add(sp.DirForward.Mul(fwd)).Add(sp.DirLeft.Mul(lft))
		offset := offsetRelative.Rotated(bodyRotation)

		// Get the floor pos
		floorPos := r.Quadruped.ShoulderVec(l).Inv().Add(globalDown).Add(r.Quadruped.ShoulderVec(l)).Add(r.Quadruped.ShoulderVec(l).Rotated(bodyRotation))

		// Set foot pos
		r.Quadruped.SetLegPosition(l, floorPos.Add(offset))
	}
}

func (r *Robot) updateStand(offset sp.Vec3) {
	for _, l := range sp.AllLegs {
		r.Quadruped.SetLegPosition(l, r.Quadruped.Legs[l].GetRestingPosition().Add(offset))
	}
}

func (r *Robot) updateBalance(bodyRotation sp.Quat) {
	globalDown := sp.DirDown.Mul(r.TrotParameters.BodyHeight).Rotated(bodyRotation)
	for _, l := range sp.AllLegs {
		floorPos := r.Quadruped.ShoulderVec(l).Inv().Add(globalDown).Add(r.Quadruped.ShoulderVec(l)).Add(r.Quadruped.ShoulderVec(l).Rotated(bodyRotation))
		r.Quadruped.SetLegPosition(l, floorPos)
	}
}
