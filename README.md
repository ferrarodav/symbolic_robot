# Robot kinematics and dynamics characterization through symbolic computation for serial manipulators

The code in this repository, once defined a serial manipulator — i.e. a robot whose links are connected in a serial fashion — computes a **complete mathematical description** through symbolic computation of the functions that characterize its kinematics and dynamics. 

Thanks to Matlab symbolic computing toolbox, the expressions are simplified (where possible) and it is possible to output an optimized matlab code numerically computing such expressions, which can be useful in **simulation** and **control** contexts.

##### Table of contents
* [Download and usage](#download-and-usage)
* [Robot definition](#robot-definition)
    * [Conventions](#conventions)
    * [Kinematics](#kinematics)
    * [Dynamics](#dynamics)
* [Computed expressions](#computed-expressions)
    * [Kinematics](#kinematics-1)
    * [Dynamics](#dynamics-1)
* [Applications examples](#applications-examples)
    * [Joint space dynamical model](#joint-space-dynamical-model)
    * [Impedance control](#impedance-control)
* [References](#references)
* [Contributing](#contributing)

### Download and usage

You can use git to download the repository and use it in your Matlab or Simulink projects:
```bash
git clone https://github.com/ferrarodav/symbolic_robot.git
```

There are three functions:
- `get_manipulator` which takes as input the definition of the manipulator structure (through the variables `DHParams`, `jointTypes` and `base`) and outputs a struct containing the symbolic definitions needed for kinematics
- `add_manipulator_dynamics` which takes as input the already computed struct and definition of the manipulator mass (through the variables `coms`, `masses`, `Is` and `g0`). It adds to the struct the symbolic definitions needed for dynamics
- `export_functions` which saves the symbolic functions to matlab code. It takes as input the robot struct, a string to prefix to the output files and a true/false flag telling the symbolic toolbox whether to optimize the code (slower while exporting)

In the `examples` folder the script `planar_robot` definse a planar manipulator – eventually the measures are perturbated to mimic construction errors – and uses the above functions to build the symbolic expressions and export them. The `plot_planar_manipulator` function uses the computed transformation matrices to draw the robot. The `plot_planar_workspace` function uses the direct kinematics to make a montecarlo simulation of the robot workspace.

### Robot definition

##### Conventions
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

##### Kinematics
- `DHParams`: a matrix where each row contains the four Denavit Hartenberg parameters of a link
- `jointTypes`: a string containing a `r` or a `p` for each link according the type of the joint connecting it to the previous one (`r` for rotational and `p` for prismatic)
- `base`: the 4x4 transformation matrix of the base reference frame (`eye(4)` if the manipulator base frame corresponds with the origin of coordinates)

##### Dynamics
- `coms`: a cell containing a 3-dimensional vector for each link pointing at its *center of mass*. Each vector must be expressed in `i`-th link reference frame (the coordinates will likely be negative, since the reference frame is centered at the end of the link)
- `masses`: a vector containing the mass of each link
- `Is`: a cell containing a 3x3 matrix for each link representing its [moment of inertia tensor](https://en.wikipedia.org/wiki/Moment_of_inertia#Inertia_tensor) (with respect to the link center of mass, the axes parallel to the `i`-th reference frame ones)
- `g0`: a 3-dimensional vector representing the gravity (usually `[0 -9.81 0]`)

### Computed expressions
The struct returned by the functions contains the parameters passed to define the robot and the symbolic expressions of the robot kinematics and dynamics.

##### Kinematics
In the struct returned by `get_manipulator`:
- `T`: a cell containing for each link the 4x4 [trasformation matrix](https://en.wikipedia.org/wiki/Transformation_matrix) of its reference frame with respect to the origin of coordinates
- `relT`: a cell containing for each link the 4x4 trasformation matrix of its reference frame with respect to the previous link reference frame
- `z`: a cell containing for each link the z axis versor with respect to the origin of coordinates
- `p`: a cell containing for each link the position vector of its reference frame with respect to the origin of coordinates
- `euler`: a cell containing for each link the euler angles describing its reference frame orientation 
- `J`: the end-effector geometric jacobian
    > ![\mathbf{v}=\begin{bmatrix}\dot{x}&\dot{y}&\dot{z}&\omega_x&\omega_y&\omega_z\end{bmatrix}=J(\mathbf{q})\cdot\mathbf{\dot{q}}](https://latex.codecogs.com/svg.latex?%5Cmathbf%7Bv%7D%3D%5Cbegin%7Bbmatrix%7D%5Cdot%7Bx%7D%26%5Cdot%7By%7D%26%5Cdot%7Bz%7D%26%5Comega_x%26%5Comega_y%26%5Comega_z%5Cend%7Bbmatrix%7D%3DJ%28%5Cmathbf%7Bq%7D%29%5Ccdot%5Cmathbf%7B%5Cdot%7Bq%7D%7D)
- `Ja`: the end-effector analytical jacobian. Unlike the geometric jacobian, this is integrable since the rotation is expressed with the euler coordinates derivatives instead of the angular velocities (which, on the other hand, have a clear physical meaning)
    > ![\mathbf{\dot{x}}=\frac{\mathrm{d}}{\mathrm{d}t}\begin{bmatrix}x&y&z&\phi&\theta&\psi\end{bmatrix}=J_A(\mathbf{q})\cdot\mathbf{\dot{q}}](https://latex.codecogs.com/svg.latex?%5Cmathbf%7B%5Cdot%7Bx%7D%7D%3D%5Cfrac%7B%5Cmathrm%7Bd%7D%7D%7B%5Cmathrm%7Bd%7Dt%7D%5Cbegin%7Bbmatrix%7Dx%26y%26z%26%5Cphi%26%5Ctheta%26%5Cpsi%5Cend%7Bbmatrix%7D%3DJ_A%28%5Cmathbf%7Bq%7D%29%5Ccdot%5Cmathbf%7B%5Cdot%7Bq%7D%7D)
- `jacobian`: the expression of the geometric jacobian with respect to a generic point in the space

##### Dynamics
In the struct returned by `add_manipulator_dynamics`:
- `Jp`: a cell containing for each link its center of mass geometric jacobian
- `g`: vector of the generalised torque imposed by gravity on each joint
- `B`: `N x N` *inertia matrix*  (N being the number of links). It encodes the inertia around each joint axis and the interaction between couples of joints. The kinetic energy is given by:
    > ![\mathcal{T}=\frac{1}{2}\mathbf{\dot{q}}^TB(\mathbf{q})\mathbf{\dot{q}}](https://latex.codecogs.com/svg.latex?%5Cmathcal%7BT%7D%3D%5Cfrac%7B1%7D%7B2%7D%5Cmathbf%7B%5Cdot%7Bq%7D%7D%5ETB%28%5Cmathbf%7Bq%7D%29%5Cmathbf%7B%5Cdot%7Bq%7D%7D)
- `C`: `N x N x N` tensor representing the *centrifugal and Coriolis effects*, also called the Christoffel symbols of the first type. The torque given by this term `c` can be computed contracting the tensor:
    > ![c_{i}(\mathbf{q},\mathbf{\dot{q}})=\sum\limits_{j=1}^{N}{\sum\limits_{k=1}^{N}{c_{ijk}(\mathbf{q})\cdot\dot{q}_j\cdot\dot{q}_k}}](https://latex.codecogs.com/svg.latex?c_%7Bi%7D%28%5Cmathbf%7Bq%7D%2C%5Cmathbf%7B%5Cdot%7Bq%7D%7D%29%3D%5Csum%5Climits_%7Bj%3D1%7D%5E%7BN%7D%7B%5Csum%5Climits_%7Bk%3D1%7D%5E%7BN%7D%7Bc_%7Bijk%7D%28%5Cmathbf%7Bq%7D%29%5Ccdot%5Cdot%7Bq%7D_j%5Ccdot%5Cdot%7Bq%7D_k%7D%7D)
- `Ja_dot`: the derivative of the analytical Jacobian with respect to time. Useful for control in the operational space

##### Substitute numerical values
Beside exporting the expressions to code with `export_functions`, the Matlab `subs` command can be used to substitute the variables. To enable this, the following symbolic variables objects are in the struct returned:
- `q`: vector of `N` variables representing the joint coordinates, used in every returned expression
- `q_dot`: vector of `N` variables representing the time derivative of the joint coordinates, used in the `Ja_dot` expression
- `point`: vector of 3 variables, representing the coordinates of a generic point in the space, used in the `jacobian` expression. It can be used to compute the geometric jacobian of a specific point of the robot: you should substitute the numeric point coordinates (with respect to the origin of coordinates frame), and — if the point belongs to the `i`-th link — zero-out the columns from the `i+1`-th on (change of successive joints coordinates don't move the point). Look at how `Jp` is obtained inside the `add_manipulator_dynamics` to see an example

### Applications examples

The expressions computed by the functions in this repository can be used in a variety of ways. Like in the simple code examples, to draw the robot points or sample the workspace. Uses of the dynamics include simulating a robot model and controlling the robot, here there are some example of the equation the output expresions fit in.

##### Joint space dynamical model
This equation defines the dynamical behaviour of a generical manipulator obtained through the Lagrangian which can be used to simulate it:

![B(\mathbf{q})\cdot\mathbf{\ddot{q}}+\mathbf{c}(\mathbf{q},\mathbf{\dot{q}})+\mathbf{g(q)}+F_{s}\cdot sng(\mathbf{\dot{q}})+F_{v}\cdot\mathbf{\dot{q}}=\mathbf{\tau}-J^{T}(\mathbf{q})\cdot\mathbf{h_{e}}](https://latex.codecogs.com/svg.latex?B%28%5Cmathbf%7Bq%7D%29%5Ccdot%5Cmathbf%7B%5Cddot%7Bq%7D%7D&plus;%5Cmathbf%7Bc%7D%28%5Cmathbf%7Bq%7D%2C%5Cmathbf%7B%5Cdot%7Bq%7D%7D%29&plus;%5Cmathbf%7Bg%28q%29%7D&plus;F_%7Bs%7D%5Ccdot%20sng%28%5Cmathbf%7B%5Cdot%7Bq%7D%7D%29&plus;F_%7Bv%7D%5Ccdot%5Cmathbf%7B%5Cdot%7Bq%7D%7D%3D%5Cmathbf%7B%5Ctau%7D-J%5E%7BT%7D%28%5Cmathbf%7Bq%7D%29%5Ccdot%5Cmathbf%7Bh_%7Be%7D%7D)
- `Fs` is the static friction coefficient (sgn is the sign function)
- `Fv` is the dynamic friction coefficient
- `tau` is the torque acting on each joint (e.g. produced by motors)
- `he` is the vector of forces and moments exerted on the environment by   the end-effector

##### Impedance control
In impedance control, once defined a desired position in the joint space `q_d`, we want to make the end-effector behave as a mass-spring-damper system with respect to the desired position in the joint space:

![\boldsymbol{\tau}_{e}=K\cdot(\mathbf{q}_d-\mathbf{q})+D\cdot(\mathbf{\dot{q}}_d-\mathbf{\dot{q}})+B(\mathbf{q})\cdot(\mathbf{\ddot{q}}_d-\mathbf{\ddot{q}})](https://latex.codecogs.com/svg.latex?%5Cboldsymbol%7B%5Ctau%7D_%7Be%7D%3DK%5Ccdot%28%5Cmathbf%7Bq%7D_d-%5Cmathbf%7Bq%7D%29&plus;D%5Ccdot%28%5Cmathbf%7B%5Cdot%7Bq%7D%7D_d-%5Cmathbf%7B%5Cdot%7Bq%7D%7D%29&plus;B%28%5Cmathbf%7Bq%7D%29%5Ccdot%28%5Cmathbf%7B%5Cddot%7Bq%7D%7D_d-%5Cmathbf%7B%5Cddot%7Bq%7D%7D%29)

We can tune as preferred the K and D, which are matrices representing the spring and damper constants. `tau_e` are the torques generated by the external forces.
The torques `tau` needed by the motors can be computed as:

![\boldsymbol{\tau}=K\cdot(\mathbf{q}_d-\mathbf{q})+D\cdot(\mathbf{\dot{q}}_d-\mathbf{\dot{q}})+B(\mathbf{q})\cdot\mathbf{\ddot{q}}_d+\mathbf{h(q,\dot{q})}](https://latex.codecogs.com/svg.latex?%5Cboldsymbol%7B%5Ctau%7D%3DK%5Ccdot%28%5Cmathbf%7Bq%7D_d-%5Cmathbf%7Bq%7D%29&plus;D%5Ccdot%28%5Cmathbf%7B%5Cdot%7Bq%7D%7D_d-%5Cmathbf%7B%5Cdot%7Bq%7D%7D%29&plus;B%28%5Cmathbf%7Bq%7D%29%5Ccdot%5Cmathbf%7B%5Cddot%7Bq%7D%7D_d&plus;%5Cmathbf%7Bh%28q%2C%5Cdot%7Bq%7D%29%7D)

With `h` being the torques given to compensate for coriolis, gravity and friction, computed as:

![\mathbf{h(q,\dot{q})}=\mathbf{c(q,\dot{q})+g(q)}+F_{s}\cdot sng(\mathbf{\dot{q}})+F_{v}\cdot\mathbf{\dot{q}}](https://latex.codecogs.com/svg.latex?%5Cmathbf%7Bh%28q%2C%5Cdot%7Bq%7D%29%7D%3D%5Cmathbf%7Bc%28q%2C%5Cdot%7Bq%7D%29&plus;g%28q%29%7D&plus;F_%7Bs%7D%5Ccdot%20sng%28%5Cmathbf%7B%5Cdot%7Bq%7D%7D%29&plus;F_%7Bv%7D%5Ccdot%5Cmathbf%7B%5Cdot%7Bq%7D%7D)

### References

The conventions, definitions and formulas used are from the following book:
> Siciliano, Bruno. **Robotics: Modelling, Planning and Control.** Springer Verlag, 2009.

### Contributing
I'd happily receive contributions to improve the repository. Here some example of things I think could be useful to the project:
- The possibility to add discrete masses to the model (needed to easily include the motor — whose rotor weight and inertia are not negligible). Ref: `eq. 7.21` of the book
- Extend the simple plot function to 3D manipulators 