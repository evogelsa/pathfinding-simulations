# 2D Robotic Motion Simulator

## Introduction

A simple 2D robot simulator. Install dependencies with `pip install -r
requirements.txt` and then run robot.py with python3. A number of run flags are
available and described below.

## Animated Motion

To run the program as just an animation try the following flags.

| Flag      | Purpose                                            | Default  |
|-----------|----------------------------------------------------|----------|
| -v        | specifies initial robot velocity                 | 5        |
| -w        | specifies initial robot angular velocity          | -3       |
| -dv       | specifies the rate of change of velocity          | 0        |
| -dw       | specifies the rate of change of angular velocity  | 0        |
| -plife    | specifies the life time of the particles in trail | 250      |
| -keyboard | enables keyboard control with arrow keys           | Disabled |

Sample configurations:

| Call                                            | Description          |
|-------------------------------------------------|----------------------|
| `py robot.py -v 3 -w -2 -dv 0.01 -plife 10000`  | Outwards spiral      |
| `py robot.py -v 10 -w -20 -dw 0.2 -plife 10000` | Spiral that reverses |
| `py robot.py -keyboard -v 5  -w 5`              | Keyboard control     |

The above sample configurations also have video recordings provided.

## PID Controller

To make the robot simulate following a track the following flags can be used to
define the PID controller.

| Flag   | Purpose                         | Default  |
|--------|---------------------------------|----------|
| -p     | tune p gain of PID controller   | 0        |
| -i     | tune i gain of PID controller   | 0        |
| -d     | tune d gain of PID controller   | 0        |
| -track | display track robot will follow | Disabled |

If your system is able to render the simulation at a constant 60 frames per
second then the following configuration provides a good example of a stable
system.

`py robot.py -track -p 72 -d 50`

However, because the simulation rate is bounded by the frame rate, a lower frame
rate means that there is less data available for the controller to process. This
makes the system naturally less stable and harder to tune properly. It also
means that the above sample configuration will likely not work when a proper
60 frames per second can not be held. This issue could be fixed by detaching the
physics engine from the rendering engine with use of threading, but due to time
constraints for this assignment, this was not possible. A sample video of the
system in a stable state is provided.

## Pathfinding

To make the robot follow a path calculated using a visibility graph and A\*
pathfinding, using the commandline argument `-pathfinding`. The optional
`-obstacles` flag can be used to specify how many obstacles to generate. Unless
the random seed is uncommented at the top of the `robot.py` file, the obstacles
and path taken will be randomly generated and recalculated each run.

When running with the `-pathfinding` flag, the program will generate a
visibility graph by connecting each vertex of an obstacle with another vertex.
Collisions between lines and obstacles are checked using the separating axis
theorem. A matrix of nodes is generated using the paths provided by the
visibility graph. This matrix is then passed into an A\* algorithm which
calculates the best path to follow.

An example video of the pathfinding can be found in the recordings folder.

### Things to be aware of

- When running the program with pathfinding enabled, it may take a moment for it
to calculate and initialize everything before the window appears.
- The robot is assumed to be a point and will appear to collide with obstacles.
  - This can be fixed by inflating the size of the obstacles as shown in the
  relevant jpg in the recordings folder.

## Probabilistic Road Map (PRM)

The program has two settings which will enable it to use a probabilistic road
map. Each of these options will generate a graph that is then used when finding
a path to the goal.

The PRM method works by randomly sampling a specified number of points. It will
then attempt to connect these points in straight lines to their k nearest
neighbors. The amount of neighbors to connect to is adjustable in the code. The
amount of points to initially sample is also configurable, but neither option is
extensively tested, and such there is no flag to change these since neither is
officially supported. However, both can be adjusted in the code by changing
the parameters passed to `geometry.prm()`. After sampling these initial points
the PRM function will start to look for a solution. If no solution is found it
will continue to sample more points in increments specified by one of the
parameters to the function until a solution is found. When the PRM method checks
for collisions it pretends that the obstacles are larger than they are in
reality. This allows the method to avoid creating a path that would come too
close to an obstacle and cause a collision as the robot traverses it.

There are two different example environments to use when enabling this method.
The first option is the `-prmscattered` flag which will generate an environment
with scattered obstacles. The PRM method will then randomly sample points and
generate a graph in which it will use to find a path to the goal.

The second option is `-prmnarrow` which creates a path with only two large
obstacles, but they are placed in a way to create a narrow path between them.
This is a particular challenge to the PRM method as it means the PRM method
essentially must get lucky with its initial point placement in order for the
gap to be traversable.

There are two videos provided which demonstrate the behavior of both of these
environments called prmscattered.mkv and prmarrow.mkv.
