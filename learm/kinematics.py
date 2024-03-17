"""
Holds all utility functions and data relevant to performing kinematics with the LeArm

I highly recommend checking out "A Note on Denavit-Hartenberg Notation in Robotics by Harvey Lipkin (2005)
This codebase was based on how the proximal variant was described in it. I am highly greatful for it as a resource.
"""
from dataclasses import dataclass
import numpy as np
import numpy.typing as npt
import math

# These are the values for the actuated joints that equate to the home configuration for the LeArm
LEARM_JOINT_OFFSETS : list[float] = [np.pi,np.pi/2.0, 0,np.pi/2.0,0,0.09]

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

def get_link0_t_linki(dh_parameters : list[DHParameters]) -> list[npt.NDArray]:
    """
    Given a list of DH parameters that start from 0 and go incrementally upward,
    return all link0_t_linki.

    For example, if you provide:
    dh_parameters = [dh_01,dh_12,dh_23]
    this will return
    world_t_linki_list = [link0_t_link0,link0_t_link1,link0_t_link2,link0_t_link3]

    Note that link0_t_link0 is always the 4x4 identity matrix (np.eye(4))
    """
    linki_t_linkj_list = [get_a_t_b(dh_parameter) for dh_parameter in  dh_parameters] # j = i+1
    world_t_linki_list = [np.eye(4)]

    for linki_t_linkj in linki_t_linkj_list:
        world_t_linkj = np.matmul(world_t_linki_list[-1], linki_t_linkj)
        world_t_linki_list.append(world_t_linkj)

    return(world_t_linki_list)

# TODO: redo actual measurements
def get_dh_parameters(q_offset_1 : float = 0 ,
                      q_offset_2 : float = 0 ,
                      q_offset_3 : float = 0 ,
                      q_offset_4 : float = 0 ,
                      q_offset_5 : float = 0 ,
                      q_offset_6 : float = 0 ,
                      ):
    """
    Returns 
    """
    dh_01 = DHParameters(
        d = 0.06985,
        theta = LEARM_JOINT_OFFSETS[0] - q_offset_1, # TODO: user input
        r = 0,
        alpha = 0,
    )


    dh_12 = DHParameters(
        d = 0,
        theta = LEARM_JOINT_OFFSETS[1] + q_offset_2, #0 TODO: user input
        r = 0.01111,
        alpha = np.pi/2.0,
    )


    dh_23 = DHParameters(
        d = 0,
        theta = LEARM_JOINT_OFFSETS[2] + q_offset_3, # TODO: user input
        r = 0.08,
        alpha = 0
    )


    dh_34 = DHParameters(
        d = 0,
        theta = LEARM_JOINT_OFFSETS[3] + q_offset_4, # TODO: user input
        r = 0.08,
        alpha = 0
    )
    dh_45 = DHParameters(
        d = 0,
        theta = LEARM_JOINT_OFFSETS[4] + q_offset_5, # TODO: user input
        r = 0,
        alpha = np.pi/2.0
    )

    dh_56 = DHParameters(
        d = LEARM_JOINT_OFFSETS[5] + q_offset_6,# TODO: user input
        theta = 0, 
        r = 0,
        alpha = 0
    )

    learm_dh_parameters = [dh_01, dh_12,dh_23,dh_34,dh_45,dh_56]
    return(learm_dh_parameters)