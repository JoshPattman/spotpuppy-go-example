package main

import (
	"encoding/json"
	"math"

	sp "github.com/JoshPattman/spotpuppy-go"
)

type HipThighKneeIK struct {
	HipLengthDown float64
	HipLengthLeft float64
	ThighLength   float64
	ShinLength    float64
}

func NewHipThighKneeIK() *HipThighKneeIK {
	return &HipThighKneeIK{}
}

func (ik *HipThighKneeIK) CalculateMotorRotations(vector sp.Vec3) []float64 {
	distance := vector.Len()
	// If adjdistance < 0 then the foot would have to be inside the hip
	adjDistance := distance - ik.HipLengthDown

	//KNEE ANGLE
	// Use cosine rule to find knee degrees
	kneeAngleRads := cosineRuleForAngle(ik.ThighLength, ik.ShinLength, adjDistance)
	// This is the knee angle in degrees centered around the leg at right angle
	kneeAngleAdjusted := sp.Degrees(kneeAngleRads) - 90

	//HIP FORWARD/BACKWARD ANGLE
	// Angle to keep the foot straight down
	// The resting rotation is when the knee joint is at 90 degrees
	kneeCounterAngleRestingRads := cosineRuleForAngle(ik.ThighLength, pythagDist(ik.ThighLength, ik.ShinLength), ik.ShinLength)
	kneeCounterAngleRads := cosineRuleForAngle(ik.ThighLength, adjDistance, ik.ShinLength)
	thighPointAngleRads := math.Atan2(vector.Dot(sp.Forward), vector.Dot(sp.Down))
	thighAngleAdjusted := sp.Degrees(kneeCounterAngleRads - kneeCounterAngleRestingRads + thighPointAngleRads)

	return []float64{0, thighAngleAdjusted, kneeAngleAdjusted}
}

var hipThighKneeIKNames = []string{"hip_left_right", "hip_forward_back", "knee"}

func (ik *HipThighKneeIK) GetMotorNames() []string {
	return hipThighKneeIKNames
}

// Loads json data into this object
func (ik *HipThighKneeIK) LoadJson(data []byte) error {
	return json.Unmarshal(data, ik)
}

// GetRestingPosition returns the position where all of the joints are at 0 degrees
func (ik *HipThighKneeIK) GetRestingPosition() sp.Vec3 {
	return sp.Down.Mul(pythagDist(ik.ThighLength, ik.ShinLength) + ik.HipLengthDown)
}

func cosineRuleForAngle(a, b, c float64) float64 {
	return math.Acos((math.Pow(a, 2) + math.Pow(b, 2) - math.Pow(c, 2)) / (2 * a * b))
}

func pythagDist(a, b float64) float64 {
	return math.Sqrt((a * a) + (b * b))
}
