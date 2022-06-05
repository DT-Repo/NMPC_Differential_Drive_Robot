## NMPC_Differential_Drive_Robot w/ Obstacle Avoidance
The project concerns solutions to optimal control problems for a differential drive robot, in particular two kind of tasks are solved, point stabilization and trajectory tracking.
Solution to these Optimal Control Problems is found by means of a Non Linear MPC strategy in order to minimize the power consumption of the robot and avoid possible obstacles in the envinroment.
Two posible direct approaches are used in the N-MPC :
* Single Shooting -> it takes into account only control actions as optimization variables.
* Multiple Shooting -> it uses also trajectory states as optimziation variables.

Furthermore a comparison between these two approaches is given. All solutions are computed by means of *fmincon* of MATLAB®.
# Single Shooting 
<img src="[https://your-image-url.type](https://github.com/DT-Repo/NMPC_Differential_Drive_Robot/blob/master/Images/single_comp.svg)" width="100" height="100">
# Multiple Shooting
An example of Trajectory Tracking Optimized is showed in the following figure:

![alt text](https://github.com/DT-Repo/NMPC_Differential_Drive_Robot/blob/master/Images/traj_trek.svg?raw=true)
 
