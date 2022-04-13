package main

import "fmt"
import "net/http"
import "encoding/json"
import "os"

func handleResponse[T InfoUpdate](res T) func(w http.ResponseWriter, r *http.Request) {
	res.OnInfoChanged()
	return func(w http.ResponseWriter, r *http.Request) {
		switch r.Method {
		case "GET":
			js, _ := json.Marshal(res)
			fmt.Fprintf(w, string(js)+"\n")
		case "POST":
			decoder := json.NewDecoder(r.Body)
			err := decoder.Decode(res)
			if err != nil {
				fmt.Println("Error parsing JSON for")
				fmt.Println(err)
			} else {
				res.OnInfoChanged()
			}
		}
	}
}

func quit(w http.ResponseWriter, r *http.Request) {
	os.Exit(0)
}

func reload(w http.ResponseWriter, r *http.Request) {
	robotRef.Load("conf")
}

func save(w http.ResponseWriter, r *http.Request) {
	robotRef.Save("conf")
}

type InfoUpdate interface {
	OnInfoChanged()
}

func (m *MovementInfo) OnInfoChanged() {
	//fmt.Println("MovementInfo was changed")
}

func (m *GaitInfo) OnInfoChanged() {
	//fmt.Println("GaitInfo was changed")
}

func (m *StateInfo) OnInfoChanged() {
	//fmt.Println("StateInfo was changed")
}

var (
	mov   = &MovementInfo{}
	state = &StateInfo{State: StateStill}
	gait  = &GaitInfo{
		StepHeight:    3,
		BodyHeight:    8,
		StepFrequency: 1,
	}
)
var robotRef *MyRobot

func startControlApi(r *MyRobot) {
	robotRef = r
	http.HandleFunc("/move", handleResponse(r.Mov))
	http.HandleFunc("/state", handleResponse(r.State))
	http.HandleFunc("/gait", handleResponse(r.Gait))
	http.HandleFunc("/quit", quit)
	http.HandleFunc("/reload", reload)
	http.HandleFunc("/save", save)
	//fmt.Println("Starting server...")
	panic(http.ListenAndServe(":10000", nil))
}
