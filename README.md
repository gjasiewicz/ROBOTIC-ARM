# **ROBOTIC-ARM**
It is a simple implementation and demo of 2D Inverse Kinematics Solver.
The goal is to determine joint angles so that the robot
reaches a specified point (end effector).
I built the general model for a robot with **n** joints.
Additionaly I implemented a demo visualizing the robot
for an end effector following a curve.

To solve this problem I used my knowledge in the following areas:

1. **Linear algebra**
* vectors and operations on them
* matrix theory
2. **Optimization algorithms**
* non-linear function minimization

I use the following libraries and toolkits:
* numpy
* scipy.optimize for finding angles of robotic arm
* matplotlib.pyplot is used for graphics
* the animated gif was created using [gifmaker](http://gifmaker.me/)

DEMO EXAMPLE

3-segment robotic arm following spiral

(https://user-images.githubusercontent.com/29254866/27431974-a4c151aa-574e-11e7-8029-06196a9baa13.gif)

4-segment robotic arm following flowerish curve

(https://user-images.githubusercontent.com/29254866/27432064-fa2e9fbc-574e-11e7-9c9b-b737b5730053.gif)

8-segment robotic arm following flowerish curve

(https://user-images.githubusercontent.com/29254866/27432081-0717d28e-574f-11e7-8108-bdbef33fd0ea.gif)
