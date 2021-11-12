# NMPC_Differential_Drive_Robot
The project concerns solutions to optimal control problems for a differential drive robot, in particular two kind of tasks are solved, point stabilization and trajectory tracking.
Solution to these Optimal Control Problems is found by means of a Non Linear MPC strategy in order to minimize the power consumption of the robot and avoid possible obstacles in the envinroment.
Two posible direct approaches are used in the N-MPC :
Single Shooting -> it takes into account only control actions as optimization variables.
Multiple Shooting -> it uses also trajectory states as optimziation variables.

Furthermore a comparison between these two approaches is given.

 