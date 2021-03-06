<h2 align="center">
  <br> MagicFlock <br>
</h2>

[![License: GPL v2](https://img.shields.io/badge/License-GPL%20v2-blue.svg)](https://www.gnu.org/licenses/old-licenses/gpl-2.0.en.html)

MagicFlock is an easy-to-use simulation platform for quadrotors. MagicFlock is a library that is intended to make quadrotors simulator easy in Gazebo simulator.
MagicFlock has an intuitive and simple interface that allows researchers to test algorithms on several quadrotors by writing only several lines of code.
In addition, the library has a set of simple examples that can use a tutorial to create and simulate your swarm.

MagicFlock differs from other libraries by the following:

* It has no dependencies on ROS
* Support data-driven approaches: such as iterative learning or Reinforcement learning
* Simple and intuitive interface
* Can be used in the Reinforcement Learning context
* Easy to install and to maintain (small amount of dependencies)

After installing MagicFlock, a set of examples can be found in `examples\` folder.
A generic example is provided as a starter guide to understand the basic simulation principle.
All examples are documented on Gitbook, can be found here.
Examples implement the algorithm needed to maintain a swarm of quadrotors from
taking off until landing. Also, to maintain the swarm intact as
long as possible.

![](3quads.gif)
![](flocking.gif)

### Dependencies
``` 
Gazebo >= 8
mlpack > 3.4 (for machine learning examples)
CMake >=3.10
Armadillo > 8.400
```

### Installation
If you have Gazebo and all other dependencies installed:

```
git clone https://github.com/shrit/MagicFlock.git
cd MagicFlock
git submodule update --init --recursive
mkdir build
cmake ../
make -j8
sudo make install
```
* Full documentation can be found here including the full installation guide
Please refer to our installation guide, to install the libraries and their dependencies.

### Paper and video 
If you are using IL4MRC in your research, please cite the following paper, can
be found here:

```
@inproceedings{shrit:hal-03133162,
  TITLE = {{Iterative Learning for Model Reactive Control: Application to autonomous multi-agent control}},
  AUTHOR = {Shrit, Omar and Filliat, David and Sebag, Michele},
  URL = {https://hal.archives-ouvertes.fr/hal-03133162},
  BOOKTITLE = {{ICARA}},
  ADDRESS = {Prague, Czech Republic},
  YEAR = {2021},
  MONTH = Feb,
  HAL_ID = {hal-03133162},
  HAL_VERSION = {v1},
  }
```

### Contribution
The library still in early-stage development with no release yet. Contributions are
very welcomed, please do not hesitate in opening issues for a problem, or sending a pull request for any thoughtful ideas. 

### License

This project is licensed under GPL license [GNU GPL v2.0](https://choosealicense.com/licenses/gpl-2.0/)

