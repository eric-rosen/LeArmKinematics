from dataclasses import dataclass
import numpy as np
import numpy.typing as npt
import math
from typing import Union


@dataclass
class DHParameters:
    """
    https://en.wikipedia.org/wiki/Denavit%E2%80%93Hartenberg_parameters

    all angles are in radians, distances in meters
    """
    # d: offset along previous z to the common normal
    d : float = 0
    # angle about previous z from old x to new x
    theta : float = 0
    #  length of the common normal. Assuming a revolute joint, this is the radius about previous z.
    r : float = 0
    # angle about common normal, from old z axis to new z axis
    alpha : float = 0

def get_trans_z_0(d : Union[float, DHParameters]) -> npt.NDArray:
    if type(d) == float:
        d = d
    elif type(d) == DHParameters:
        d = d.d
    else:
        raise ValueError(f"{type(d)} is not a valid parameter for d")
        
    trans_z_0 = np.eye(4)
    trans_z_0[2,3] = d

    return(trans_z_0)

def get_rot_z_0(theta : Union[float, DHParameters]) -> npt.NDArray:
    """
    assumes theta is in radians
    """
    if type(theta) == float:
        theta = theta
    elif type(theta) == DHParameters:
        theta = theta.theta
    else:
        raise ValueError(f"{type(theta)} is not a valid parameter for theta")
        
    rot_z_0 = np.eye(4)
    rot_z_0[0,0] = math.cos(theta)
    rot_z_0[1,1] = math.cos(theta)
    rot_z_0[0,1] = -math.sin(theta)
    rot_z_0[1,0] = math.sin(theta)

    return(rot_z_0)


def get_trans_x_1(r : Union[float, DHParameters]) -> npt.NDArray:
    if type(r) == float:
        r = r
    elif type(r) == DHParameters:
        r = r.r
    else:
        raise ValueError(f"{type(r)} is not a valid parameter for r")
        
    trans_z_1 = np.eye(4)
    trans_z_1[0,3] = r

    return(trans_z_1)

def get_rot_x_1(alpha : Union[float, DHParameters]) -> npt.NDArray:
    """
    assumes alpha is in radians
    """
    if type(alpha) == float:
        alpha = alpha
    elif type(alpha) == DHParameters:
        alpha = alpha.alpha
    else:
        raise ValueError(f"{type(alpha)} is not a valid parameter for alpha")
        
    rot_x_1 = np.eye(4)
    rot_x_1[1,1] = math.cos(alpha)
    rot_x_1[2,2] = math.cos(alpha)
    rot_x_1[1,2] = -math.sin(alpha)
    rot_x_1[2,1] = math.sin(alpha)

    return(rot_x_1)

def get_a_t_b(dh_parameter : DHParameters):
    trans_z = get_trans_z_0(dh_parameter)
    rot_z = get_rot_z_0(dh_parameter)
    trans_x = get_trans_x_1(dh_parameter)
    rot_x = get_rot_x_1(dh_parameter)

    return trans_z.dot(rot_z).dot(trans_x).dot(rot_x)

'''
The LeArm is 6DoF arm. We write the kinematic chain for the arm as follows:
world_frame -> link_1 -> link_2 -> link_3 -> link_4 -> link_5 -> link_6
where arrow_i (i \in [0,5]) represents a joint (geometric constraint) between parent link_i and child link_i+1.

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

By convention, we will ground the axes of world_frame as:
- having positive z oriented perpendicular to be facing upwards perpendicullary to the base
- having posiive x oriented parallel to the base and pointing directly away from the back of the base
- having y follows right hand rule  (in this case, pointing out "rightwards" in the frame of reference of the
world frame, parallel to the base).

The origin of the world_frame will ground as:
- having z height be equal to height of the base. (All parts of the arm in the home configuration are "above" the base)
- have x and y so that they origin intersects with the base. 

Conceptually, this grounds to a point that is in the plane of the black plate, and directly under the hole that the
first motor goes through.

We now write out the DH parameters for each pair of joints, where
dh_i is the dh parameters between link_i and link_i+1

### dh_0 / theta_0 / world_frame -> link_1
theta_0 is related to the first motor, which has an axis of rotation that is parallel to the
world_frame's z_axis. Under DH convention, this makes means that the z axis for frame attached to 
link_1 goes in the same direction as the z_axis for the world_frame.

Since the since z's are parallel, we can choose the common normal as long as in in the plane perpendiucular to the 
z axis of the world_frame/link_1, along with d (the displacement along the z axis from the 
origin of the world_frame in this case.). Here, we choose d to be the height from the black plate base
to the height blue plate base (i.e: the circular base link that the first motor controls), and make the x/y axis point
in the same direction as the world_frame when at the home joint configuration. 
'''


learm_dh_parameters = []
dh_0_d = 0.06985
dh_theta_0 = 0 # TODO: user input
dh_r_0 = 0
dh_alpha_0 = 0

"""
### dh_1 / theta_1 / link_1 -> link_2

The second servo has an axis of rotation that is parallel to the y axis of link_1's y_axis.
Therefore, the z axis for the frame attached to link_2 is parallel to link_1's y_axis,
with postive direction in the direction of the postive y of the world_frame.

the common normal for these two axes of rotations is parallel to the x axis of the world_frame,
with the x axis positively pointing towards the the front of the base since the motor is a 
little behind the center of the circular blue base.

The y axis follows right hand rule, which in this case, makes positive y point in the same
direction as positive z in the world_frame.

"""

dh_1_d = 0.017462
dh_theta_1 = 0 # TODO: user input
dh_r_1 = -0.01111
dh_alpha_1 = np.pi/2.0


learm_dh_parameters.append(DHParameters(d=dh_0_d,
                                        theta=dh_theta_0,
                                        r=dh_r_0,
                                        alpha=dh_alpha_0))


learm_dh_parameters.append(DHParameters(d=dh_1_d,
                                        theta=dh_theta_1,
                                        r=dh_r_1,
                                        alpha=dh_alpha_1))

print(learm_dh_parameters)


link_2_p_testp = np.array([0,0,0,1])
world_t_link1 = get_a_t_b(learm_dh_parameters[0])
link1_t_link2 = get_a_t_b(learm_dh_parameters[1])
print(f"world_t_link: {world_t_link1}")
print(f"link1_t_link2: {link1_t_link2}")
print(world_t_link1.dot(link1_t_link2.dot(link_2_p_testp)))