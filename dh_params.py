'''
The LeArm is 6DoF arm. We write the kinematic chain for the arm as follows:
world_frame -> link1 -> link2 -> link3 -> link4 -> link5 -> link6
where arrow ->_i (i \in [0,5]) represents a joint (geometric constraint) between parent link_i and child link_i+1.

From this, we can see that there are 6 arrows, which means that the 
arm has 6 joints (or degrees of freedom (DoF)).

If we specify the 6 values of the joint positions, we have fully specified
 the confogiration of the robot (the positions of the links in 3D space are fully determined).

We represent the current configuration as theta = (theta_1, ..., theta_6), where theta_i represents arrow_i 

We want to attach a 3D frame to each link, and transform 3d poses in one frame to another as a function of
theta (the current joint configuration). Doing so is called forward kinematics. To do this, we will use
the DH-parameters of the LeArm 6DoF robot arm.

https://en.wikipedia.org/wiki/Denavit%E2%80%93Hartenberg_parameters
'''