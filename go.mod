module spotpuppy-proto

require github.com/JoshPattman/spotpuppy-go v0.0.5

require (
	github.com/googolgl/go-i2c v0.1.1 // indirect
	github.com/googolgl/go-pca9685 v0.1.5 // indirect
	github.com/sirupsen/logrus v1.8.1 // indirect
	github.com/tarm/serial v0.0.0-20180830185346-98f6abe2eb07 // indirect
	github.com/westphae/quaternion v0.0.0-20210908005042-fa06d546065c // indirect
	golang.org/x/sys v0.0.0-20211015200801-69063c4bb744 // indirect
)

//replace github.com/JoshPattman/spotpuppy-go => ../spotpuppy-go

go 1.18
