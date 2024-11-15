# VMRobotControl.jl

![VMRobotControl.jl logo](docs/src/assets/Logo.svg)

VMRobotControl allows you to build dynamic controllers for your robot as intuitive
virtual mechanisms.
You design your controller by building a virtual mechanism.
You connect the virtual mechanism to the robot through compliant elements (like springs and dampers).
The control action is automatically translated to the robot actuators (desired joint forces/torques).

This package is designed for building, simulating and controlling robots
using passivity-based controllers represented by virtual mechanisms.
This is in the tradition of *virtual model control* and *energy shaping/damping injection*.
For a defined robot geometry, you can select specific *coordinates* and connect
 *components* to shape the robot dynamic properties.
properties.
This provides a simple way to program complex robot behaviour incrementally, from mechanical 
primitives.

# Installation

Julia must be installed, I recommend using [juliaup](https://github.com/JuliaLang/juliaup). `VMRobotControl.jl` is only tested on the latest julia version. 

Then, from the Julia REPL run:
```julia
using Pkg
Pkg.add("VMRobotControl")
```
To load the package, use the command:
```julia
using VMRobotControl
```

## Developer installation

Then clone [the github repository](https://github.com/Cambridge-Control-Lab/VMRobotControl.jl), and navigate to the repos's root folder in a terminal, e.g.

```
git clone git@github.com:Cambridge-Control-Lab/VMRobotControl.jl.git
```

Run julia and hit the `]` key to enter the `Pkg` context for managing packages/environments. 
Run the following sequence of commands:
```
registry update
develop ./VMRobotControl.jl
```
This updates the package registry from online, then marks the folder `VMRobotControl.jl` as a
package under development.
To check things are working, while still in the `Pkg` context, you can run the packages self-tests
by activating the packages environment, and running test
```
activate ./VMRobotControl.jl
test
```

# Manually building documentation

Activate the docs environment, from the VMRobotControl.jl folder:
```
activate docs
add DifferentialEquations, Documenter, FileIO, ForwardDiff, GLMakie, Literate, LiveServer, MeshIO, StaticArrays
```
Then, run the contents of `./docs/make.jl`...
```julia
include("./docs/make.jl")
```
and host them using `LiveServer.jl`:
```julia
include("./docs/make.jl")
```