## NMPC_Differential_Drive_Robot w/ Obstacle Avoidance
The project concerns solutions to optimal control problems for a differential drive robot, in particular two kind of tasks are solved, point stabilization and trajectory tracking.<br>
Solution to these Optimal Control Problems is found by means of a Non Linear MPC strategy in order to minimize the power consumption of the robot and avoid possible obstacles in the environment.<br>
Two possible direct approaches are used with the N-MPC strategy :
* Single Shooting -> it takes into account only control actions as optimization variables.
 <img src="https://github.com/DT-Repo/NMPC_Differential_Drive_Robot/blob/master/Images/single_shoot.PNG">
 
* Multiple Shooting -> it uses also trajectory states as optimziation variables.
<img src="https://github.com/DT-Repo/NMPC_Differential_Drive_Robot/blob/master/Images/multiple.PNG">

Furthermore a comparison between these two approaches is given. All solutions are computed by means of *fmincon* of MATLAB®.
# Point Stabilization
|Single Shooting |Multiple Shooting |
| ------------- | ------------- |
|<img src="https://github.com/DT-Repo/NMPC_Differential_Drive_Robot/blob/master/Images/single_comp.svg" width="500" height="500"> | <img src="https://github.com/DT-Repo/NMPC_Differential_Drive_Robot/blob/master/Images/multi_comp.svg" width="500" height="500"> |


# Trajectory Tracking
An example of a Trajectory Tracking, solved with Single Shooting approach, is showed in the following figure:
<img src="https://github.com/DT-Repo/NMPC_Differential_Drive_Robot/blob/master/Images/traj_trek.svg?raw=true" width="500" height="500">

 
