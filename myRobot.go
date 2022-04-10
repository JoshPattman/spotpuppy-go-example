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
	Mov *MovementInfo
	Gait *GaitInfo
	State *StateInfo

}

const (
	StateStill = "still"
    StateStanding = "standing"
    StateTrot = "trotting"
)

func NewRobot() *MyRobot {
	q := sp.NewQuadrupedWithExtraMotors(sp.NewDirectMotorIKGenerator(), sp.NewPCAMotorController(), []string{"neck"})
	return &MyRobot{
		Quad:   q,
		Sensor: sp.NewArduinoRotationSensor("/dev/ttyUSB0"),
		CS:     sp.NewRollPitchCoordinateSystem(),
		T:      0,
		LT:     time.Now(),
		Mov: &MovementInfo{},
		Gait: &GaitInfo{
			StepHeight: 2,
	        BodyHeight: q.Legs[sp.LegFrontLeft].GetRestingPosition().Y,
	        StepFrequency: 1,
		},
		State: &StateInfo{State:StateStill},
	}
}
func NewDummyRobot() *MyRobot {
	q := sp.NewQuadrupedWithExtraMotors(sp.NewDirectMotorIKGenerator(), sp.NewDummyMotorController(), []string{"neck"})
	return &MyRobot{
		Quad:   q,
		Sensor: sp.NewDummyRotationSensor(),
		CS:     sp.NewRollPitchCoordinateSystem(),
		T:      0,
		LT:     time.Now(),
		Mov: &MovementInfo{},
		Gait: &GaitInfo{
			StepHeight: 2,
	        BodyHeight: q.Legs[sp.LegFrontLeft].GetRestingPosition().Y,
	        StepFrequency: 1,
		},
		State: &StateInfo{State:StateStill},
	}
}

func (r *MyRobot) Update() {
	// Timekeeping
	r.T += time.Since(r.LT).Seconds() * r.Gait.StepFrequency
	r.LT = time.Now()
	clkA := math.Mod(r.T, 1.0)
	clkB := math.Mod(r.T+0.5, 1.0)
	// Update rotation sensor
	roll, pitch := r.Sensor.GetRollPitch()
	r.CS.SetRollPitch(roll, pitch)
	hasFallen := math.Abs(roll) < 30 && math.Abs(pitch) < 30
	if r.State.State == StateTrot && !hasFallen{
		// Custom walking code
		snA := math.Sin(clkA * 3.1415 * 2)
		snB := math.Sin(clkB * 3.1415 * 2)
		snA = math.Min(snA, 0)
		snB = math.Min(snB, 0)
		stepA := r.CS.TD(sp.DirUp.Mul(snA).Mul(r.Gait.StepHeight))
		stepB := r.CS.TD(sp.DirUp.Mul(snB).Mul(r.Gait.StepHeight))
		straightDown := r.CS.TD(sp.DirDown.Mul(r.Gait.BodyHeight))
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
	} else if r.State.State == StateStanding && !hasFallen{
		// Stand but keep feet on the ground
		straightDown := r.CS.TD(sp.DirDown.Mul(r.Gait.BodyHeight))
		for _, l := range sp.AllLegs {
			floorPos := r.Quad.ShoulderVec(l).Add(straightDown).Add(r.CS.TD(r.Quad.ShoulderVec(l).Inv()))
			r.Quad.SetLegPosition(l, floorPos)
		}
	} else if !hasFallen{
		// Stand still
		localDown := sp.DirDown.Mul(r.Gait.BodyHeight)
		for _, l := range sp.AllLegs {
			r.Quad.SetLegPosition(l, localDown)
		}
	} else{
		// Stand still at half height
		localDown := sp.DirDown.Mul(r.Gait.BodyHeight/2)
		for _, l := range sp.AllLegs {
			r.Quad.SetLegPosition(l, localDown)
		}
	}
	// Update quad
	r.Quad.Update()
}

type MovementInfo struct{
    VelocityFwd float64 `json:"vel_fwd"`
    VelocityLft float64 `json:"vel_lft"`
    RotationClkwise float64 `json:"rot_clk"`
}
type GaitInfo struct{
    StepHeight float64 `json:"step_height"`
    BodyHeight float64 `json:"body_height"`
    StepFrequency float64 `json:"step_frequency"`
}
type StateInfo struct{
    State string `json:"state"`
}
