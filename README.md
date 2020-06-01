-> SPOQ <-

[![License: GPL v2](https://img.shields.io/badge/License-GPL%20v2-blue.svg)](https://www.gnu.org/licenses/old-licenses/gpl-2.0.en.html)
SPOQ is an easy to use Simulation Platform fOr Quadrotors. SPOQ is a library that intended to make quadrotors simulator easy in Gazebo simulator.
SPOQ has inituative and simple interface that allows researcher to test algorithms on several quadrotors by writing only several lines of code.
In additions, the library has a set of simple exaples that can be use a tutorial in order to create and simulate you swarm.

SPOQ differs from other libraries by the following:

* It has no dependencies on ROS
* Support data driven approaches 
* Simple and inituative interface
* Can be used in Reinforcement learning context
* Easy to install and to maintain (small amount of dependencies)

After installing SPOQ, a set of examples can be found in `examples\` folder.
A generic example is provided as starter guide to understand the basic simulation principle.
All examples are documented on gitbook, can be found here.
Examples implement the algorithm needed to maintain a swarm of quadrotors from
taking off until landing. The objective is to maintain the swarm intact as
long as possible.

### Dependencies
``` 
Gazebo >= 8
mlpack (for machine learning examples)
CMake >=3.10

```

### Installation
If you have Gazebo and all other dependencies installed:

```
cd SPOQ
mkdir build
cmake ../
make -j32

```
* Full documentation can be found here including full installation guide
Please refer to our installation guide, to install the libraries and its dependencies.

### Paper and video 
If you are using IL4MRC in your research, please cite the following paper, can
be found here:
```

```

### Contribution
The library still in early stage development with no release yet. Contributions are
very welcomed, please do not hesitate in opening issues for problem, or a
sending pull request for any thoughtful ideas. Contribution guide can be
found here.

### License

This project is licensed under GPL license [GNU GPL v2.0](https://choosealicense.com/licenses/gpl-2.0/)

