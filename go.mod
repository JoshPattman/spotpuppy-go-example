module spotpuppy-proto

require github.com/JoshPattman/spotpuppy-go v0.0.0
require github.com/JoshPattman/spotpuppy-go/pca9685 v0.0.0
require github.com/JoshPattman/spotpuppy-go/arduinompu v0.0.0

require (
	github.com/tarm/serial v0.0.0-20180830185346-98f6abe2eb07 // indirect
	github.com/westphae/quaternion v0.0.0-20210908005042-fa06d546065c // indirect
	golang.org/x/sys v0.0.0-20220406163625-3f8b81556e12 // indirect
)

replace github.com/JoshPattman/spotpuppy-go => ./../spotpuppy-go

replace github.com/JoshPattman/spotpuppy-go/arduinompu => ./../spotpuppy-go/arduinompu

replace github.com/JoshPattman/spotpuppy-go/pca9685 => ./../spotpuppy-go/pca9685

go 1.17
