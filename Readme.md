# [Robotics 2] Dynamic and Redundancy toolkit
The DnR toolkit is based on [Robotics2Tools by MoSamArafat](https://github.com/MoSamArafat/Robotics2Tools). It includes useful scripts for the midterm / exam of Robotics 2.

## Introduction
Here are written the standards of the toolbox.
### Denavit-Hartenberg Table Convention
The Toolkit uses the standard convention presented during lectures of Robotics1 and Robotics2:

| alpha | d | a | theta |
|:-:|:-:|:-:|:-:|
| ... | ... | ... | ...|

## Functions
Here the functions are briefly described

### Kinematics
#### Geometry
- __h2r__  : Extracts the rotation matrix from a given homogeneous transformation matrix
- __h2tr__ : Extracts the rotation matrix AND the offset vector from a given h.trasformation matrix
#### Denavit-Hartenberg
- __dh_step__        : Compute a single link transformation, given a DH parameter row
- __dh_transform_m__ : Compute the h.transformation matrix given the DH table, a source joint and a destination joint

### Dynamics
- __compute_sym_m__ : Computes the Symbolic Inertia matrix M(q, q_dot)
- __compute_num_m__ : Computes the Numerical Inertia matrix M(q, q_dot) through the Euler-Newton method.
- __compute_christoffel__ : Computes the coriolis/centrifugal factorization through christoffel symbols
