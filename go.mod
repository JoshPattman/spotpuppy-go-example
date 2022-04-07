module spotpuppy-proto

require github.com/JoshPattman/spotpuppy-go v0.0.0

require github.com/westphae/quaternion v0.0.0-20210908005042-fa06d546065c // indirect

replace github.com/JoshPattman/spotpuppy-go => ./../spotpuppy-go

replace github.com/JoshPattman/spotpuppy-go/arduinompu => ./../spotpuppy-go/arduinompu

replace github.com/JoshPattman/spotpuppy-go/pca9685 => ./../spotpuppy-go/pca9685

go 1.17
