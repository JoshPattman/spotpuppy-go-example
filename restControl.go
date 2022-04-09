package main

import "fmt"
import "net/http"
import "encoding/json"

func handleResponse[T InfoUpdate](res T)func (w http.ResponseWriter, r *http.Request){
    res.OnInfoChanged()
    return func (w http.ResponseWriter, r *http.Request){
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
            } else{
                res.OnInfoChanged()
            }
        }
    }
}



type InfoUpdate interface{
    OnInfoChanged()
}


func (m *MovementInfo) OnInfoChanged(){
    //fmt.Println("MovementInfo was changed")
}

func (m *GaitInfo) OnInfoChanged(){
    //fmt.Println("GaitInfo was changed")
}


func (m *StateInfo) OnInfoChanged(){
    //fmt.Println("StateInfo was changed")
}

var (
    mov = &MovementInfo{}
    state = &StateInfo{State:StateStill}
    gait = &GaitInfo{
        StepHeight: 3,
        BodyHeight: 8,
        StepFrequency: 1,
    }
)

func startControlApi(r *MyRobot) {
    http.HandleFunc("/move", handleResponse(r.Mov))
    http.HandleFunc("/state", handleResponse(r.State))
    http.HandleFunc("/gait", handleResponse(r.Gait))
    //fmt.Println("Starting server...")
    panic(http.ListenAndServe(":10000", nil))
}
