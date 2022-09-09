package main

import (
	"encoding/json"
	"flag"
	"fmt"
	"os"
	"runtime"
	"time"

	sp "github.com/JoshPattman/spotpuppy-go"
	"github.com/simulatedsimian/joystick"
)

var mode string

var configFile string

var generateConfig bool

func main() {
	// Parse cli
	flag.StringVar(&mode, "m", "stand", "the operation for the robot to do (one of: 'trot-in-place', 'trot-forwards', 'stand', 'stand-tall', 'stand-front-left', 'balance', 'trot-js', 'point')")
	flag.StringVar(&configFile, "f", "conf-dummy", "the filename of the robot config")
	flag.BoolVar(&generateConfig, "g", false, "if this is true then instead of running the robot, a config file will instead be generated")
	flag.Parse()

	// create robot with appropriate peripherals
	fmt.Println("Creating robot")
	var r *Robot
	if runtime.GOARCH == "amd64" {
		r = NewRobot(sp.NewDummyMotorController(), sp.NewDummyRotationSensor())
		fmt.Println("Created dummy robot (as we are not on a rpi)")
	} else {
		sensor := sp.NewRawArduinoRotationSensor()
		//sensor := sp.NewDummyRotationSensor()
		r = NewRobot(
			sp.NewPCAMotorController(),
			sensor,
		)
		fmt.Println("Created PCA and arduino robot")
	}
	if generateConfig {
		os.MkdirAll(configFile, 0700)
		if err := r.Quadruped.SaveToFile(configFile + "/config.json"); err != nil {
			panic(err)
		}
		jsGait, _ := json.MarshalIndent(r.TrotParameters, "", "\t")
		if err := os.WriteFile(configFile+"/gait.json", jsGait, 0644); err != nil {
			panic(err)
		}
		jsRot, _ := json.MarshalIndent(r.RotationSensor, "", "\t")
		if err := os.WriteFile(configFile+"/rot.json", jsRot, 0644); err != nil {
			panic(err)
		}
	} else {
		// setup the robots mode
		useJoystick := false
		switch mode {
		case "trot-in-place":
			r.Mode = ModeTrot
		case "trot-forwards":
			r.Mode = ModeTrot
			r.VelFwd = 3
		case "stand":
			r.Mode = ModeStand
		case "stand-tall":
			r.Mode = ModeStandTall
		case "stand-front-left":
			r.Mode = ModeStandFL
		case "balance":
			r.Mode = ModeBalance
		case "trot-js":
			r.Mode = ModeStand
			useJoystick = true
		case "point":
			r.Mode = ModePoint
		default:
			panic("Mode not recognised")
		}
		fmt.Println("Starting robot with mode", mode)

		var js joystick.Joystick
		if useJoystick {
			fmt.Println("Connecting to joystick")
			js = ConnectToJS()
		}

		// load the quadruped settings from file
		if err := r.Quadruped.LoadFromFile(configFile + "/config.json"); err != nil {
			panic(err)
		}
		// load the gait settings from file
		if data, err := os.ReadFile(configFile + "/gait.json"); err != nil {
			panic(err)
		} else {
			if err = json.Unmarshal(data, r.TrotParameters); err != nil {
				panic(err)
			}
		}
		// load the rotation sensor settings from file
		if data, err := os.ReadFile(configFile + "/rot.json"); err != nil {
			panic(err)
		} else {
			if err = json.Unmarshal(data, r.RotationSensor); err != nil {
				panic(err)
			}
			r.RotationSensor.Setup()
			// Make the robot stand whilst calibrating
			m := r.Mode
			r.Mode = ModeStand
			r.Update()
			r.Mode = m
			fmt.Println("Calibrating sensor")
			time.Sleep(time.Second)
			r.RotationSensor.Calibrate()
		}
		// begin updating the robot velocities in the background
		if useJoystick {
			go RunJSController(js, r)
		}
		// update the robot at 100 times per second
		fmt.Println("Updating robot")
		time.Sleep(time.Second)
		ups := sp.NewUPSTimer(50)
		for {
			r.Update()
			ups.WaitForNext()
		}
	}
}
