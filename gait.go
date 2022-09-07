package main

// This file contains code for generating a foot offset from the floor when given certain parameters

import (
	"fmt"
	"math"
)

func factorial(n int) int {
	factVal := 1
	for i := 1; i <= n; i++ {
		factVal *= i
	}
	return factVal
}

func nCr(n, r int) float64 {
	return float64(factorial(n)) / float64(factorial(r)*factorial(n-r))
}

func bezier(xs []float64, t float64) float64 {
	if !(t >= 0 && t <= 1) {
		return math.NaN()
	}
	v := 0.0
	n := len(xs) - 1
	for i := range xs {
		v += nCr(n, i) * math.Pow(1-t, float64(n-i)) * math.Pow(t, float64(i)) * xs[i]
	}
	return v
}

func xyBezierTransformed(xs []float64, xStart, xEnd, t float64) float64 {
	return bezier(xs, (t-xStart)/(xEnd-xStart))
}

type GaitParameters struct {
	// Value between 0 and 1 (usuall 0.4 works) that determines how much x movement happens after the foot has stopped contacting the ground
	ExtendHorizontal float64 `json:"horizontal-extension"`
	// Value in units (cm) that says how high the tallest point in the step is
	StepHeight float64 `json:"step-height"`
	// Value between 0 and 1 where 0 means the leg spends 100 percent of the time on the floor and 1 means 100 percent of the time in the air (usually 0.25-0.5)
	Airtime float64 `json:"airtime"`
}

// t MUST be between 0 and 1
func (g *GaitParameters) VerticalOffsetFromFloor(pushForce, t float64) float64 {
	{
		// Upwards stroke of leg
		yUp := xyBezierTransformed([]float64{
			-pushForce, -pushForce,
			g.StepHeight, g.StepHeight,
		}, 0, g.Airtime/2, t)
		if !math.IsNaN(yUp) {
			return yUp
		}
	}
	{
		// Downwards stroke of leg
		yDown := xyBezierTransformed([]float64{
			g.StepHeight, g.StepHeight,
			0, 0,
		}, g.Airtime/2, g.Airtime, t)
		if !math.IsNaN(yDown) {
			return yDown
		}
	}
	{
		// Grounded stroke of leg
		yGround := xyBezierTransformed([]float64{
			0, 0,
			-pushForce, -pushForce,
		}, g.Airtime, 1, t)
		if !math.IsNaN(yGround) {
			return yGround
		}
	}
	panic(fmt.Sprint("This should not have happened (all leg y strokes were NaN): ", t))
}

func (g *GaitParameters) HorizontalOffset(stepLength, t float64) float64 {
	maxRunoutY := g.Airtime / (4 - 4*g.Airtime)
	runout := g.ExtendHorizontal * maxRunoutY
	runoutX := 2 * runout * (1 - g.Airtime)
	{
		// Grounded stroke
		yGround := xyBezierTransformed([]float64{
			0,
			1,
		}, g.Airtime, 1, t)*2 - 1
		if !math.IsNaN(yGround) {
			return yGround * stepLength
		}
	}
	{
		// Leaving ground extenstion
		yGroundLeaveExt := xyBezierTransformed([]float64{
			1,
			1 + runout, 1 + runout,
		}, 0, runoutX, t)*2 - 1
		if !math.IsNaN(yGroundLeaveExt) {
			return yGroundLeaveExt * stepLength
		}
	}
	{
		// Return in air stroke
		yAir := xyBezierTransformed([]float64{
			1 + runout, 1 + runout,
			-runout, -runout,
		}, runoutX, g.Airtime-runoutX, t)*2 - 1
		if !math.IsNaN(yAir) {
			return yAir * stepLength
		}
	}
	{
		// Returning to ground extenstion
		yGroundReturnExt := xyBezierTransformed([]float64{
			-runout, -runout,
			0,
		}, g.Airtime-runoutX, g.Airtime, t)*2 - 1
		if !math.IsNaN(yGroundReturnExt) {
			return yGroundReturnExt * stepLength
		}
	}
	panic(fmt.Sprint("This should not have happened (all leg x strokes were NaN): ", t))
}
