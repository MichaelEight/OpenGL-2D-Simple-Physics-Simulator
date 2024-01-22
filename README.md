# OpenGL 2D C++ Simple Physics Simulator
Experimental physics simulator. *Simple* as in "creating different laws of physics, which will neither work perfectly nor perform optimally".
There's no nice, easy-to-use editor right now and everything needs to be added manually via code (find **Main Loop**). 

**NOTE:** Constants (like e.g. gravitational G) had to be changed to reflect the size scale in the simulator.

## Features
- create objects (circles) with different radius, mass, color, position, linear velocity
- apply force(s) to each object. Mass affects the effect of the force (`a = F/m`)
- enable mutual gravity (`F = G * (m1 * m2)/r^2`)
- handle collisions (both walls and with other objects)
- handle conservation of momentum
- draw path behind the object

## TODO
- handle rotation, angular velocity, conservation of angular momentum
- create fixed-length joints between elements to keep them apart and figure out angular momentum of complex objects 

## Example Simulation
![obraz](https://github.com/MichaelEight/OpenGL-2D-Simple-Physics-Simulator/assets/56772277/a85bdd18-d727-4906-8835-aa8237bcb01b)
![obraz](https://github.com/MichaelEight/OpenGL-2D-Simple-Physics-Simulator/assets/56772277/e8d1e4b2-4996-4380-a335-f8e9dc5e2921)
