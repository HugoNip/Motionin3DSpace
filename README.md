# Motionin3DSpace
## Introductin
The fundamental operations in the SLAM.

## Requirements
### Eigen Package
#### Install
```
sudo apt-get install libeigen3-dev
```
#### Search Installing Location
```
sudo updatedb
locate eigen3
```

default location "/usr/include/eigen3"


### OpenGL Package (use pangolin)
#### Download
https://github.com/stevenlovegrove/Pangolin

#### Install the dependency for pangolin (mainly the OpenGL)
```
sudo apt-get install libglew-dev
```

#### Compile and Install pangolin
```
cd [path-to-pangolin]
mkdir build
cd build
cmake ..
make 
sudo make install 
```

## Compile this Project
```
mkdir build
cd build
cmake ..
make 
```

## Run
```
./eigenMatrix
./coordinateTransform
./plotTrajectory
./useGeometry
./visualizeGeometry
```
## Reference
[Source](https://github.com/HugoNip/slambook2/tree/master/ch3)
