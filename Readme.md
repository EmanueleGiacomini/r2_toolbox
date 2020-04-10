# [Robotics 2] Dynamic and Redundancy toolkit
The DnR toolkit is based on [Robotics2Tools by MoSamArafat](https://github.com/MoSamArafat/Robotics2Tools). It includes useful scripts for the midterm / exam of Robotics 2.

## Overview
Here are written the standards of the toolbox.

### Terminology
In case you don't understand some terminology of the toolbox, you may find (and also request) explanation here:

- __rci__ : position wrt RFi of a link's CoM 
- __roc__ : position wrt RF0 of a link's CoM

### Denavit-Hartenberg Table Convention
The Toolkit uses the standard convention presented during lectures of Robotics1 and Robotics2:

| alpha | d | a | theta |
|:-:|:-:|:-:|:-:|
| ... | ... | ... | ...|

In order to build the table, let's see how one row generates a new frame RF1 from RF0:

__KEEP IN MIND__ that you need to use the right hand rule to generate the first frame, then I suggest thinking at how you wish to place the next frame, and then build up a chain of rototranslations that can get you there. The 4 parameters of the table lets you play with this rototranslation in the following way:
* Take RF1 and place it with the same orientation on RF0 position
* Move RF1 on z0 axis by d, then apply a counter clockwise rotation by theta on RF1 from z0
* Move RF1 on x1 axis by a, then apply a counter clockwise rotation by alpha on RF1 from x1

Remember that only x and z axis are used during the rototranslation, and that (d, theta) are applied onto z0 while (a, alpha) are applied on x1.

## Functions
Here the functions are briefly described

### Kinematics
#### Geometry
- __h2r__  : Extracts the rotation matrix from a given homogeneous transformation matrix
- __h2tr__ : Extracts the rotation matrix AND the offset vector from a given h.trasformation matrix
#### Denavit-Hartenberg
- __dh_step__        : Compute a single link transformation, given a DH parameter row
- __dh_transform_m__ : Compute the h.transformation matrix given the DH table, a source joint and a destination joint
#### Generic
- __compute_roc__ : Compute the RoC vector given the DH table and a link index

### Dynamics
- __compute_sym_m__ : Computes the Symbolic Inertia matrix M(q, q_dot)
- __compute_num_m__ : Computes the Numerical Inertia matrix M(q, q_dot) through the Euler-Newton method.
- __compute_christoffel__ : Computes the coriolis/centrifugal factorization through christoffel symbols
- __compute_gravity__ : Compute the gravity vector for the arm

## Generalized Robot computation
- __planar_robot__ : Dynamic model for a generic planar robot.
- __spatial_robot__ : Dynamic model for a generic spatial robot.

### Examples
Some examples are also provided on operativity of the toolbox:
- *planar_2r*  : Dynamic model for 2R planar robot.
- *planar_prp* : Dynamic model for PRP planar robot.
- *planar_pprr* : Dynamic model for PPRR planar robot.
- *spatial_3r* : Dynamic model for 3R spatial robot.
