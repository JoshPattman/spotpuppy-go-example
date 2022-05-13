package main

import (
	"encoding/json"
	"io/ioutil"
	"math"
	"time"

	sp "github.com/JoshPattman/spotpuppy-go"
)

type MyRobot struct {
	Quad   *sp.Quadruped
	Sensor sp.RotationSensor
	T      float64
	LT     time.Time
	Mov    *MovementInfo
	Gait   *GaitInfo
	State  *StateInfo
}

const (
	StateStill    = "still"
	StateStanding = "standing"
	StateTrot     = "trotting"
)

func NewRobotWith(mc sp.MotorController, rs sp.RotationSensor) *MyRobot {
	q := sp.NewQuadrupedWithExtraMotors(sp.NewDirectMotorIKGenerator(), mc, []string{"neck"})
	return &MyRobot{
		Quad:   q,
		Sensor: rs,
		T:      0,
		LT:     time.Now(),
		Mov:    &MovementInfo{},
		Gait: &GaitInfo{
			StepHeight:     2,
			BodyHeight:     q.Legs[sp.LegFrontLeft].GetRestingPosition().Y,
			StepFrequency:  1,
			LeanMultiplier: 2.0 / 45.0,
		},
		State: &StateInfo{State: StateStill},
	}
}
func NewRobot() *MyRobot {
	return NewRobotWith(sp.NewPCAMotorController(), sp.NewArduinoRotationSensor("/dev/ttyUSB0", sp.AxesRemap{X: "x", Y: "y", Z: "z"}))
}
func NewDummyRobot() *MyRobot {
	return NewRobotWith(sp.NewDummyMotorController(), sp.NewDummyRotationSensor())
}
func (r *MyRobot) Load(name string) {
	r.Quad.LoadFromFile(name + ".json")
	file, _ := ioutil.ReadFile(name + "_gait.json")
	_ = json.Unmarshal(file, r.Gait)

}
func (r *MyRobot) Save(name string) {
	r.Quad.SaveToFile(name + ".json")
	file, _ := json.MarshalIndent(r.Gait, "", " ")
	_ = ioutil.WriteFile(name+"_gait.json", file, 0644)
}

func (r *MyRobot) Update() {
	// Timekeeping
	r.T += time.Since(r.LT).Seconds() * r.Gait.StepFrequency
	r.LT = time.Now()
	clkA := math.Mod(r.T, 1.0)
	clkB := math.Mod(r.T+0.5, 1.0)
	// Update rotation sensor. In this model we do not care about the robots yaw so we remove that
	bodyRotation := r.Sensor.GetQuaternion().NoYaw()
	hasFallen := sp.DirUp.AngleTo(sp.DirUp.Rotated(bodyRotation)) > 30
	if r.State.State == StateTrot && !hasFallen {
		// Custom walking code
		// Walking offsets and the horizontal/vertical functions that move the foot throught the step
		stepXOffsetA, stepYOffsetA := getWalkingOffsets(clkA, 1)
		stepXOffsetB, stepYOffsetB := getWalkingOffsets(clkB, 1)
		// stepY is the vertical component of a foots movement
		stepYA := sp.DirUp.Mul(stepYOffsetA).Mul(r.Gait.StepHeight).Rotated(bodyRotation)
		stepYB := sp.DirUp.Mul(stepYOffsetB).Mul(r.Gait.StepHeight).Rotated(bodyRotation)
		// stepMv is the horizontal component of a foots movement
		stepDir := sp.DirForward.Mul(r.Mov.VelocityFwd / r.Gait.StepFrequency).Add(sp.DirLeft.Mul(r.Mov.VelocityLft / r.Gait.StepFrequency))
		stepMvA := stepDir.Mul(stepXOffsetA)
		stepMvB := stepDir.Mul(stepXOffsetB)
		// lean is the horizontal offset for legs in response to the robot tilting
		cgOffset := sp.DirUp.Rotated(bodyRotation).Sub(sp.DirUp)
		leanFwd := sp.DirForward.Mul(cgOffset.X * r.Gait.LeanMultiplier)
		leanLft := sp.DirLeft.Mul(cgOffset.Z * r.Gait.LeanMultiplier)
		// StraightDown is the vector that goes straight down from the robots cg to the floor
		straightDown := sp.DirDown.Mul(r.Gait.BodyHeight).Rotated(bodyRotation)
		for _, l := range sp.AllLegs {
			// Find the position of the foot on a flat floor
			floorPos := r.Quad.ShoulderVec(l).Add(straightDown).Add(r.Quad.ShoulderVec(l).Inv().Rotated(bodyRotation))
			// Find the step offset for this moment in time
			var step sp.Vec3
			if l == sp.LegFrontLeft || l == sp.LegBackRight {
				step = stepYA.Add(stepMvA).Rotated(bodyRotation)
			} else {
				step = stepYB.Add(stepMvB).Rotated(bodyRotation)
			}
			// Add everything together
			r.Quad.SetLegPosition(l, floorPos.Add(step).Add(leanFwd).Add(leanLft))
		}
	} else if r.State.State == StateStanding && !hasFallen {
		// Stand but keep feet on the ground
		straightDown := sp.DirDown.Mul(r.Gait.BodyHeight).Rotated(bodyRotation)
		for _, l := range sp.AllLegs {
			shoulderVec := r.Quad.ShoulderVec(l)
			floorPos := shoulderVec.Add(straightDown).Add(shoulderVec.Inv().Rotated(bodyRotation))
			r.Quad.SetLegPosition(l, floorPos)
		}
	} else if !hasFallen {
		// Stand still
		localDown := sp.DirDown.Mul(r.Gait.BodyHeight)
		for _, l := range sp.AllLegs {
			r.Quad.SetLegPosition(l, localDown)
		}
	} else {
		// Stand still at half height
		localDown := sp.DirDown.Mul(r.Gait.BodyHeight / 2)
		for _, l := range sp.AllLegs {
			r.Quad.SetLegPosition(l, localDown)
		}
	}
	// Update quad
	r.Quad.Update()
}

type MovementInfo struct {
	VelocityFwd     float64 `json:"vel_fwd"`
	VelocityLft     float64 `json:"vel_lft"`
	RotationClkwise float64 `json:"rot_clk"`
}
type GaitInfo struct {
	StepHeight     float64 `json:"step_height"`
	BodyHeight     float64 `json:"body_height"`
	StepFrequency  float64 `json:"step_frequency"`
	LeanMultiplier float64 `json:"lean_multiplier"`
}
type StateInfo struct {
	State string `json:"state"`
}

// This can be visualised with thi graph i made: https://www.desmos.com/calculator/qa4qzd2wy3
func getWalkingOffsets(t float64, ratio float64) (float64, float64) {
	if ratio > 1 {
		ratio = 1
	} else if ratio < 0 {
		ratio = 0
	}
	y := 0.0
	x := 0.0
	m := 2.0 / (ratio - 2)
	if t > 0.5*ratio {
		y = 0
		x = (m * t) - m
	} else {
		y = math.Sin(t * 3.14159 * 2 / ratio)
		x = 2 * t / ratio
	}
	return x, y
}
