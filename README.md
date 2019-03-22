# QCopter

[![Build Status](https://travis-ci.com/shrit/QCopters.svg?token=ERsskMDsU2icrkrWzj6i&branch=master)](https://travis-ci.com/shrit/QCopters) [![Build Status: develop](https://travis-ci.com/shrit/QCopters.svg?token=ERsskMDsU2icrkrWzj6i&branch=develop)](https://travis-ci.com/shrit/QCopters) [![License: GPL v2](https://img.shields.io/badge/License-GPL%20v2-blue.svg)](https://www.gnu.org/licenses/old-licenses/gpl-2.0.en.html)

QCopters is a software used to control a swarm of quadcopters in
Gazebosim, it has the capacity to control several quads at the same
time. It uses machine learning algorithms to maintain the intial
formation of quadcopters


# Installation:

To compile the project:

```bash
mkdir build && cd build
```
```bash
cmake ../
```
```bash
make
```

Please verify that you have installed the following dependecies on your
machine:

# Dependencies:

[Boost libraries](https://www.boost.org/)  
[ncurses](https://www.gnu.org/software/ncurses/)  
[Gazebo and Gazebo API](http://gazebosim.org/)  
[DronecodeSDK](https://github.com/Dronecode/DronecodeSDK)  

# License

This project is licensed under GPL license [GNU GPL v2.0](https://choosealicense.com/licenses/gpl-2.0/)




