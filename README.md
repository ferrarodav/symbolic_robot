# Serial manipulator kinematics and dynamics characterization through symbolic computation

The code in this repository, once defined a serial manipulator — i.e. a robot whose links are connected in a serial fashion — computes a **complete mathematical description** through symbolic computation of the functions that characterize its kinematics and dynamics. Thanks to Matlab symbolic computing toolbox, the expressions are simplified (where possible) and it is possible to output an optimized matlab code numerically computing such expressions, which can be useful in **simulation** and **control** contexts.

### Table of contents

* [Installation and usage](#installation_and_usage)
* [Conventions](#conventions)
* [get_manipulator.m](#?)
    * [Robot definition](#)
    * [Computed expressions](#)
    * [Dynamical model](#)
* [Usage example and plot functions](#usage_example_and_plot_functions)
* [References](#references)
* [Contributing](#contributing)

### Installation and usage

Download the content of the repo to use it in your projects.
```bash
git clone git://www.github.com/ferrarodav/symbolic_robot
```
An example of how 

### Conventions

The [Denavit Hartenberg (DH) convention](https://en.wikipedia.org/wiki/Denavit%E2%80%93Hartenberg_parameters) is used to represent the manipulator.
Each link is represented by four parameters which indicate how to transform the reference frame in order to get from a joint to the next:
- *d*: offset along previous z axis to the common normal
- *theta*: angle about previous z axis, from old x axis to new x axis
- *r*: length of the common normal. Assuming a revolute joint, this is the radius about previous z axis
- *alpha*: angle about common normal, from old z axis to new z axis

The `i`-th joint is positioned at the `i-1`-th link reference frame and moves the `i`-th link by rotating `q_i` radiants along the z axis — if rotational — or by displacing `q_i` meters along the z axis — if prismatic.
Note that the `i`-th link reference frame is positioned at its end.
The *base* reference frame is the one where the first joint is positioned.
The *end-effector* reference frame is the last link reference frame.

### `get_manipulator.m`
This file contains the main matlab function which takes the manipulator definition as input and outputs a struct containing the symbolical expressions.

`function [robot] = get_manipulator(DHParams, jointTypes, coms, masses, Is, g0, base)`:
- `DHParams`: a matrix where each row contains the four Denavit Hartenberg parameters of a link
- `jointTypes`: a string containing a `r` or a `p` for each link according the type of the joint connecting it to the previous one (`r` for rotational and `p` for prismatic)
- `coms`: a cell containing a 3-dimensional vector for each link pointing at its *center of mass*. Each vector must be expressed in `i`-th link reference frame (the coordinates will likely be negative, since the reference frame is centered at the end of the link)
- `masses`: a vector containing the mass of each link
- `Is`: a cell containing a 3x3 matrix for each link representing its [*inertia matrix*](https://en.wikipedia.org/wiki/Moment_of_inertia#Inertia_tensor) (with respect to the link center of mass, the axes parallel to the `i`-th reference frame ones)
- `g0`: a 3-dimensional vector representing the gravity (usually `[0 -9.81 0]`)
- `base`: the 4x4 transformation matrix of the base reference frame (`eye(4)` if the manipulator base frame corresponds with the origin of coordinates)

The function returns a struct containing:
- the passed parameters
- `q`: vector of *symbolic variables* matlab objects, they represent each joint coordinate and can be used to substitute with numerical values the `q` symbols in the symbolic expressions
- `T`: a cell containing for each link the 4x4 [trasformation matrix](https://en.wikipedia.org/wiki/Transformation_matrix) of its reference frame with respect to the origin of coordinates
- `relT`: a cell containing for each link the 4x4 trasformation matrix of its reference frame with respect to the previous link reference frame
- `z`: a cell containing for each link the z axis versor with respect to the origin of coordinates
- `p`: a cell containing for each link the position vector of its reference frame with respect to the origin of coordinates
- `euler`: a cell containing for each link the euler angles describing its reference frame orientation 
- `J`: the end-effector geometric jacobian
    > ![\mathbf{v}=\begin{bmatrix}\dot{x}&\dot{y}&\dot{z}&\omega_x&\omega_y&\omega_z\end{bmatrix}=J(\mathbf{q})\cdot\mathbf{\dot{q}}](https://latex.codecogs.com/svg.latex?%5Cmathbf%7Bv%7D%3D%5Cbegin%7Bbmatrix%7D%5Cdot%7Bx%7D%26%5Cdot%7By%7D%26%5Cdot%7Bz%7D%26%5Comega_x%26%5Comega_y%26%5Comega_z%5Cend%7Bbmatrix%7D%3DJ%28%5Cmathbf%7Bq%7D%29%5Ccdot%5Cmathbf%7B%5Cdot%7Bq%7D%7D)
- `Ja`: the end-effector analytical jacobian. Unlike the geometric jacobian, this is integrable since the rotation is expressed with the euler coordinates derivatives instead of the angular velocities (which, on the other hand, have a clear physical meaning)
    > ![\mathbf{\dot{x}}=d\begin{bmatrix}x&y&z&\phi&\theta&\psi\end{bmatrix}/dt=J_A(\mathbf{q})\cdot\mathbf{\dot{q}}](https://latex.codecogs.com/svg.latex?%5Cmathbf%7B%5Cdot%7Bx%7D%7D%3Dd%5Cbegin%7Bbmatrix%7Dx%26y%26z%26%5Cphi%26%5Ctheta%26%5Cpsi%5Cend%7Bbmatrix%7D/dt%3DJ_A%28%5Cmathbf%7Bq%7D%29%5Ccdot%5Cmathbf%7B%5Cdot%7Bq%7D%7D)
- `Jp`: a cell containing for each link its center of mass geometric jacobian
- `jacobian`: the expression of the geometric jacobian with respect to a generic point in the space
- `point`: vector of 3 *symbolic variables* matlab objects, they represent the coordinates of a generic point in the space and can be used together with `jacobian` to compute the geometric jacobian of a specific point of the robot. You should substitute the specific point coordinates (with respect to the origin of coordinates frame), and — if the point belongs to the `i`-th link — zero-out the columns from the `i+1`-th on (change of successive joints coordinates don't move the point). Look at how `Jp` is obtained inside the function to see an example
- `g`: vector of the generalised torque imposed by gravity on each joint
- `B`: `N x N` matrix  (N being the number of links) representing the inertia around each joint axis and between couples of joints. The kinetic energy is given by:
    > ![]()
- `C`: `N x N x N` tensor representing the centrifugal and Coriolis effects. The C matrix used in the dynamical model can be obtained as:
    > ![]()
- `qt`: 
- `q_dot`: 
- `Ja_dot`: the derivative of the analytical Jacobian with respect to time

These quantities can be put together in the dynamical model of the manipulator, which can be used to simulate it:

![B(\mathbf{q})\cdot\mathbf{\ddot{q}}+C(\mathbf{q},\mathbf{\dot{q}})\cdot\mathbf{\dot{q}}+\mathbf{g(q)}+F_{s}\cdot sng(\mathbf{\dot{q}})+F_{v}\cdot\mathbf{\dot{q}}=\mathbf{\tau}-J^{T}(\mathbf{q})\cdot\mathbf{h_{e}}](https://latex.codecogs.com/svg.latex?B%28%5Cmathbf%7Bq%7D%29%5Ccdot%5Cmathbf%7B%5Cddot%7Bq%7D%7D&plus;C%28%5Cmathbf%7Bq%7D%2C%5Cmathbf%7B%5Cdot%7Bq%7D%7D%29%5Ccdot%5Cmathbf%7B%5Cdot%7Bq%7D%7D&plus;%5Cmathbf%7Bg%28q%29%7D&plus;F_%7Bs%7D%5Ccdot%20sng%28%5Cmathbf%7B%5Cdot%7Bq%7D%7D%29&plus;F_%7Bv%7D%5Ccdot%5Cmathbf%7B%5Cdot%7Bq%7D%7D%3D%5Cmathbf%7B%5Ctau%7D-J%5E%7BT%7D%28%5Cmathbf%7Bq%7D%29%5Ccdot%5Cmathbf%7Bh_%7Be%7D%7D)
- Fs is the static friction coefficient (sgn is the sign function)
- Fv is the dynamic friction coefficient
- Tau is the generalized torque acting on each joint (e.g. produced by motors)
- he is the generalized force acting on the end-effector

### Usage example and plot functions


### References

The conventions, definitions and formulas used are from the following book:
> Siciliano, Bruno. **Robotics: Modelling, Planning and Control.** Springer Verlag, 2009.

### Contributing
I'd happily receive contribution to improve the repository. Here some things I think could be useful to the project:
- The possibility to add discrete masses to the model (needed to easily include the motor — whose rotor weight and inertia are not negligible). Ref: `eq. 7.21` of the book
- Extend the simple plot function to 3D manipulators to ease debugging of the manipulator definition and showcase ?