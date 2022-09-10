package main

// This is the new robot logic. I have not finished writing it yet

import (
	"math"
	"time"

	sp "github.com/JoshPattman/spotpuppy-go"
)

type RobotMode int

const (
	// For tuning the servos to have the correct 0 positions by having all servos go to their resting pos
	ModeStand RobotMode = iota
	// For flipping the knee and thigh servos so that they are the correct way round by putting all feet lower than their resting pos
	ModeStandTall
	// For flipping the thigh and hip servos so that they are the correct way round by pointing all legs towards the front left
	ModeStandFL
	// For checking the gyro is working by always keeping the legs on a flat plane no matter what rotation the body is at
	ModeBalance
	// For moving around with a balanced walk
	ModeTrot
	// For checking the gyro alignment is correct by pointing the legs towards the direction the robot is leaning
	ModePoint
)

type TrotParameters struct {
	GaitParameters     *GaitParameters `json:"gait-gen-params"`
	StepFrequency      float64         `json:"step-freq"`
	BodyHeight         float64         `json:"body-height"`
	DirectAcceleration float64         `json:"direct-acceleration"`
	PushPIFwd          *PIController   `json:"push-force-pi-fwd"`
	PushPILft          *PIController   `json:"push-force-pi-lft"`
}

type Robot struct {
	Quadruped                  *sp.Quadruped
	RotationSensor             sp.RotationSensor
	t                          float64
	lastUpdate                 time.Time
	TrotParameters             *TrotParameters
	Mode                       RobotMode
	VelFwd, VelLft             float64
	smoothVelFwd, smoothVelLft float64
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
	dt := time.Since(r.lastUpdate).Seconds()
	r.t += dt * r.TrotParameters.StepFrequency
	r.lastUpdate = time.Now()

	// Rotation
	bodyRotation := r.RotationSensor.GetQuaternion()
	bodyRotationCorrected := bodyRotation.NoYaw()
	//fmt.Println(bodyRotationCorrected.Apply(sp.Up))
	hasFallen := sp.Up.AngleTo(sp.Up.Rotated(bodyRotation)) > 30
	if !hasFallen {
		switch r.Mode {
		case ModeStand:
			r.updateStand(sp.NewVector3(0, 0, 0))
		case ModeStandTall:
			r.updateStand(sp.Down.Mul(3))
		case ModeStandFL:
			r.updateStand(sp.Forward.Mul(3).Add(sp.Left.Mul(3)))
		case ModeBalance:
			r.updateBalance(bodyRotationCorrected)
		case ModeTrot:
			r.updateTrot(bodyRotationCorrected, dt)
		case ModePoint:
			r.updateModePoint(bodyRotationCorrected)
		}
	} else {
		r.updateStand(sp.Up.Mul(3))
	}
	r.Quadruped.Update()
}

// Calculates the position of a foot such that no matter how the body is rotated, the foot stays in the same place in global space.
// It does this by adding these vectors: (shoulder -> robot center) + (robot center -> position below robot center in global space) + (robot center -> shoulder in global space)
func calcFloorPos(q *sp.Quadruped, leg string, bodyRotation sp.Quat, height float64) sp.Vec3 {
	svRobot := q.ShoulderVec(leg)
	svGlobal := svRobot.Rotated(bodyRotation.Inv())
	downGlobal := sp.Down.Mul(height).Rotated(bodyRotation.Inv())
	return svRobot.Inv().Add(downGlobal).Add(svGlobal)
}

// The basic idea of this trot algo is as follows:
// -> Each leg has push force proportional to how far the robot is tilting towards it
// -> Generate floor offsets from gait.go file with the push forces descibed above. Each diagonal leg pair has the same timing for its gait (both pairs are offset by half a cycle from each other)
// -> Calculate the position on the floor directly below each shoulder (this takes into account the body rotation)
// -> Add the floor offsets to this position on the floor. Each leg should now be trotting
func (r *Robot) updateTrot(bodyRotation sp.Quat, dt float64) {
	// Clocks
	clkA := math.Mod(r.t, 1.0)
	clkB := math.Mod(r.t+0.5, 1.0)
	clks := make(map[string]float64)
	clks[sp.LegFrontLeft], clks[sp.LegBackRight] = clkA, clkA
	clks[sp.LegFrontRight], clks[sp.LegBackLeft] = clkB, clkB

	// Move the actual velocities towards the targets
	r.smoothVelFwd = moveTowardsFloat(r.smoothVelFwd, r.VelFwd, r.TrotParameters.DirectAcceleration*dt)
	r.smoothVelLft = moveTowardsFloat(r.smoothVelLft, r.VelLft, r.TrotParameters.DirectAcceleration*dt)

	// Calculating lean amounts
	globalUp := sp.Up.Rotated(bodyRotation)
	rbtLeanedFwd, rbtLeanedLft := globalUp.Dot(sp.Forward), globalUp.Dot(sp.Left)

	// Dynamic params
	r.TrotParameters.PushPIFwd.Current, r.TrotParameters.PushPILft.Current = rbtLeanedFwd, rbtLeanedLft
	leanFwd, leanLft := -r.TrotParameters.PushPIFwd.NextAdjustment(), -r.TrotParameters.PushPILft.NextAdjustment()

	// Convert to gait
	for _, l := range sp.AllLegs {
		// Get the gait offset
		gaitUp := r.TrotParameters.GaitParameters.VerticalOffsetFromFloor(0, clks[l])
		gaitOffset := sp.Up.Mul(gaitUp)

		floorPos := sp.Down.Mul(r.TrotParameters.BodyHeight)

		r.Quadruped.SetLegPosition(l, floorPos.Add(gaitOffset).Add(sp.Forward.Mul(leanFwd)).Add(sp.Left.Mul(leanLft)))
	}
}

func (r *Robot) updateStand(offset sp.Vec3) {
	for _, l := range sp.AllLegs {
		r.Quadruped.SetLegPosition(l, r.Quadruped.Legs[l].GetRestingPosition().Add(offset))
	}
}

func (r *Robot) updateBalance(bodyRotation sp.Quat) {
	for _, l := range sp.AllLegs {
		floorPos := calcFloorPos(r.Quadruped, l, bodyRotation, r.TrotParameters.BodyHeight)
		r.Quadruped.SetLegPosition(l, floorPos)
	}
}
func (r *Robot) updateModePoint(bodyRotation sp.Quat) {
	globalUp := sp.Up.Rotated(bodyRotation)
	pointForward := globalUp.Dot(sp.Forward)
	pointLeft := globalUp.Dot(sp.Left)
	offset := sp.Forward.Mul(pointForward).Add(sp.Left.Mul(pointLeft))
	for _, l := range sp.AllLegs {
		r.Quadruped.SetLegPosition(l, r.Quadruped.Legs[l].GetRestingPosition().Add(offset.Mul(5)))
	}
}

func moveTowardsFloat(current, target, velocity float64) float64 {
	d := target - current
	if d == 0 {
		return target
	}
	sign := d / math.Abs(d)
	if d/sign < velocity {
		return target
	}
	return sign*velocity + current
}
