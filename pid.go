package main

// The reason I am choosing to use a PI not a PID controller is that the accelerometer readings are very shaky, which can easily confuse PID d terms

import "time"

type PIController struct {
	Current, Target float64
	Kp, Ki          float64
	i               float64
	lastUpdate      time.Time
}

func (pi *PIController) NextAdjustment() float64 {
	// Timekeeping
	now := time.Now()
	dt := now.Sub(pi.lastUpdate).Seconds()
	pi.lastUpdate = now
	d := pi.Target - pi.Current
	p := d * pi.Kp
	pi.i += d * pi.Ki * dt
	return p + pi.i
}

func (pi *PIController) Reset() {
	pi.i = 0
	pi.lastUpdate = time.Now()
}

func NewPIController(Kp, Ki float64) *PIController {
	return &PIController{
		0, 0,
		Kp, Ki,
		0,
		time.Now(),
	}
}
