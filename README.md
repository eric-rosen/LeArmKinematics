# LeArmKinematics
Python library for kinematics of the LeArm 6-DOF robotic arm.

References for the arm:
https://www.amazon.com/LewanSoul-Programmable-Feedback-Parameter-Programming/dp/B074T6DPKX
https://github.com/ccourson/xArmServoController

# Kinematic notes
The LeArm is 6DoF arm. We write the kinematic chain for the arm as follows:

`world_frame -> link_1 -> link_2 -> link_3 -> link_4 -> link_5 -> link_6`

where the ith `->` (arrow) represents a joint (geometric constraint) between parent link_i and child link_i+1.

From this, we can see that there are 6 arrows, which means that the 
arm has 6 joints (or degrees of freedom (DoF)).

If we specify the 6 values of the joint positions, we have fully specified
 the confogiration of the robot (the positions of the links in 3D space are fully determined).

We represent the current configuration as theta = (theta_0, ..., theta_5), where theta_i represents arrow_i 

We want to attach a 3D frame to each link, and transform 3d poses in one frame to another as a function of
theta (the current joint configuration). Doing so is called forward kinematics. To do this, we will use
the DH-parameters of the LeArm 6DoF robot arm.

https://en.wikipedia.org/wiki/Denavit%E2%80%93Hartenberg_parameters

For grounding, we will refer to "the base" as the black plate that the controller and arm both sit on. Note that
all motors for the arm lie "above" the base.

## World Frame

By convention, we will ground the axes of world_frame as:
- having positive z oriented perpendicular to be facing upwards perpendicullary to the base
- having posiive x oriented parallel to the base and pointing directly away from the back of the base
- having y follows right hand rule  (in this case, pointing out "leftwards" in the frame of reference of the
world frame, parallel to the base).

The origin of the world_frame will ground as:
- having z height be equal to height of the base. (All parts of the arm in the home configuration are "above" the base)
- have x and y so that they origin intersects with the base. 

Conceptually, this grounds to a point that is in the plane of the black plate, and directly under the hole that the
first motor goes through.

We now write out the DH parameters for each pair of joints, where
dh_i is the dh parameters between link_i and link_i+1

## dh_0 / theta_0 / world_frame -> link_1

theta_0 is related to the first motor, which has an axis of rotation that is parallel to the
world_frame's z_axis. Under DH convention, this makes means that the z axis for frame attached to 
link_1 goes in the same direction as the z_axis for the world_frame.

Since the since z's are parallel, we can choose the common normal as long as it is in the plane perpendiucular to the 
z axis of the world_frame/link_1, along with d (the displacement along the z axis from the 
origin of the world_frame in this case.). Here, we choose d to be the height from the black plate base
to the height blue plate base (i.e: the circular base link that the first motor controls), and make the x/y axis point
in the same direction as the world_frame when at the home joint configuration. 

## dh_1 / theta_1 / link_1 -> link_2

The second servo has an axis of rotation that is parallel to the y axis of link_1's y_axis.
Therefore, the z axis for the frame attached to link_2 is parallel to link_1's y_axis,
with postive direction in the direction of the positive y of the world_frame.

the common normal for these two axes of rotations is parallel to the x axis of the world_frame,
with the x axis positively pointing towards the the back of the base (negative x direction in world frame) since the motor is a 
little behind the center of the circular blue base and the common normal points away.

The y axis follows right hand rule, which in this case, makes positive y point in the same
direction as positive z in the world_frame.

## dh_2 / theta_2 / link_2 -> link_3

axis of rotation for motor 3 faces in same direction as motor 2, 
so z's are the same. Common normal (x-axis) goes in the negative
z direction of the world-frame, and goes through the origin of
link_2 and link_3 frames. positive y axis in pointing in the 
negative x direction of world frame.

We  need to calculate the length of the common r
and rotate common normal by pi/2 around z