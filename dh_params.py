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

learm_dh_parameters = []
dh_0_d = 0.06985
dh_0_theta = 0 # TODO: user input
dh_0_r = 0
dh_0_alpha = 0

dh_1_d = 0.017462
dh_1_theta = 0 # TODO: user input
dh_1_r = -0.01111 # TODO: confirm this is okay to make negative and isn't a mistake?
dh_1_alpha = np.pi/2.0

dh_2_d = 0
dh_2_theta = np.pi/2 # TODO: user input
dh_2_r = 0.1063625
dh_2_alpha = 0 


learm_dh_parameters.append(DHParameters(d=dh_0_d,
                                        theta=dh_0_theta,
                                        r=dh_0_r,
                                        alpha=dh_0_alpha))


learm_dh_parameters.append(DHParameters(d=dh_1_d,
                                        theta=dh_1_theta,
                                        r=dh_1_r,
                                        alpha=dh_1_alpha))

learm_dh_parameters.append(DHParameters(d=dh_2_d,
                                        theta=dh_2_theta,
                                        r=dh_2_r,
                                        alpha=dh_2_alpha))

print(learm_dh_parameters)


_p_testp = np.array([0,0,0,1])
world_t_link1 = get_a_t_b(learm_dh_parameters[0])
link1_t_link2 = get_a_t_b(learm_dh_parameters[1])
link2_t_link3 = get_a_t_b(learm_dh_parameters[2])
print(f"world_t_link1: {world_t_link1}")
print(f"link1_t_link2: {link1_t_link2}")
print(f"link2_t_link3: {link2_t_link3}")
print(world_t_link1.dot(link1_t_link2.dot(_p_testp)))
print(world_t_link1.dot(link1_t_link2.dot(link2_t_link3.dot(_p_testp))))