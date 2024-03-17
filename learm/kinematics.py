"""
Holds all utility functions and data relevant to performing kinematics with the LeArm

I highly recommend checking out "A Note on Denavit-Hartenberg Notation in Robotics by Harvey Lipkin (2005)
This codebase was based on how the proximal variant was described in it. I am highly greatful for it as a resource.
"""
from dataclasses import dataclass
import numpy as np
import numpy.typing as npt
import math
from typing import Union

LEARM_JOINT_OFFSETS : list[float] = [np.pi,np.pi/2.0, 0,np.pi/2.0,0,0.1524] # gripper confirmed twice!
'''These are the values for the actuated joints that equate to the home configuration for the LeArm.
These are meant to be constants and should not be touched'''

LEARN_JOINT_MIN_MAX = [
    [LEARM_JOINT_OFFSETS[0]-np.pi/2.0,LEARM_JOINT_OFFSETS[0]+np.pi/2.0],
    [LEARM_JOINT_OFFSETS[1]-np.pi/2.0,LEARM_JOINT_OFFSETS[1]+np.pi/2.0],
    [LEARM_JOINT_OFFSETS[2]-np.pi/2.0,LEARM_JOINT_OFFSETS[2]+np.pi/2.0],
    [LEARM_JOINT_OFFSETS[3]-np.pi/2.0,LEARM_JOINT_OFFSETS[3]+np.pi/2.0],
    [LEARM_JOINT_OFFSETS[4]-np.pi/2.0,LEARM_JOINT_OFFSETS[4]+np.pi/2.0],
    [LEARM_JOINT_OFFSETS[5],LEARM_JOINT_OFFSETS[5]+0.03175]
]
'''
The min/max of the joint ranges for each joint.
'''

@dataclass
class DHParameters:
    """
    These Denavit–Hartenberg (DH) parameters follow the proximal variant

    all angles are in radians
    distances are standardly in meters
    """
    d : float = 0
    """d: offset along previous z to the common normal"""
    theta : float = 0
    """angle about previous z from old x to new x"""
    r : float = 0
    """length of the common normal. Assuming a revolute joint, this is the radius about previous z."""
    alpha : float = 0
    """angle about common normal, from old z axis to new z axis"""

def get_a_t_b(dh_parameter : DHParameters) -> npt.NDArray:
    """
    Gets the SE(3) that represents where frame b is in
    frame a (conceptually, a is parent frame, b is child frame).
    """
    mat_a = np.eye(4)
    mat_a[1,0] = dh_parameter.r
    mat_a[2,2] = math.cos(dh_parameter.alpha)
    mat_a[3,3] = math.cos(dh_parameter.alpha)
    mat_a[2,3] = -math.sin(dh_parameter.alpha)
    mat_a[3,2] = math.sin(dh_parameter.alpha)

    mat_b = np.eye(4)
    mat_b[3,0] = dh_parameter.d
    mat_b[1,1] = math.cos(dh_parameter.theta)
    mat_b[2,2] = math.cos(dh_parameter.theta)
    mat_b[1,2] = -math.sin(dh_parameter.theta)
    mat_b[2,1] = math.sin(dh_parameter.theta)
    return(np.matmul(mat_a,mat_b))

def get_link0_t_linki(dh_parameters : Union[list[DHParameters],list[float]]) -> list[npt.NDArray]:
    """
    Given a list of DH parameters that start from 0 and go incrementally upward,
    return all link0_t_linki.

    For example, if you provide:
    dh_parameters = [dh_01,dh_12,dh_23]
    this will return
    world_t_linki_list = [link0_t_link0,link0_t_link1,link0_t_link2,link0_t_link3]

    Note that link0_t_link0 is always the 4x4 identity matrix (np.eye(4))
    """
    if len(dh_parameters) == 0:
        raise RuntimeError(f"dh_parameters can not be empty, was length {len(dh_parameters)}")
    else:
        if isinstance(dh_parameters[0], float):
            dh_parameters = get_dh_parameters(*dh_parameters)

    linki_t_linkj_list = [get_a_t_b(dh_parameter) for dh_parameter in  dh_parameters] # j = i+1
    world_t_linki_list = [np.eye(4)]

    for linki_t_linkj in linki_t_linkj_list:
        world_t_linkj = np.matmul(world_t_linki_list[-1], linki_t_linkj)
        world_t_linki_list.append(world_t_linkj)

    return(world_t_linki_list)

def get_dh_parameters(q_0 : float = LEARM_JOINT_OFFSETS[0],
                      q_1  : float = LEARM_JOINT_OFFSETS[1] ,
                      q_2  : float = LEARM_JOINT_OFFSETS[2] ,
                      q_3  : float = LEARM_JOINT_OFFSETS[3] ,
                      q_4  : float = LEARM_JOINT_OFFSETS[4] ,
                      q_5  : float = LEARM_JOINT_OFFSETS[5] ,
                      ):
    """
    Returns dh parameters, with each q
    """
    dh_01 = DHParameters(
        d = 0.03175,
        theta = q_0, # revolute joint
        r = 0,
        alpha = 0,
    )


    dh_12 = DHParameters(
        d = 0,
        theta = q_1, # revolute joint
        r = 0.009525, # got this twice, nice!
        alpha = np.pi/2.0,
    )


    dh_23 = DHParameters(
        d = 0,
        theta = q_2, # revolute joint
        r = 0.0762,
        alpha = 0
    )


    dh_34 = DHParameters(
        d = 0,
        theta = q_3, # revolute joint
        r = 0.0889,
        alpha = 0
    )
    dh_45 = DHParameters(
        d = 0,
        theta = q_4, # revolute joint
        r = 0,
        alpha = np.pi/2.0
    )

    dh_56 = DHParameters(
        d = q_5,# prismatic joint
        theta = 0, 
        r = 0,
        alpha = 0
    )

    learm_dh_parameters = [dh_01, dh_12,dh_23,dh_34,dh_45,dh_56]
    return(learm_dh_parameters)