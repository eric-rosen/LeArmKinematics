from dataclasses import dataclass
import numpy as np
import numpy.typing as npt
import math
from typing import Union
import matplotlib.pyplot as plt
import itertools

@dataclass
class DHParameters:
    """
    Proximal Variant
    I recommend checking out "A Note on Denavit-Hartenberg Notation in Robotics by Harvey Lipkin (2005)

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

    #print(f"mat_a: { mat_a}")
    #print(f"mat_b: {mat_b}")

    return(np.matmul(mat_a,mat_b))

def draw_frame_axes(world_t_frame : npt.NDArray, ax) -> None:
    world_p_origin = world_t_frame[1:,0]
    world_r_origin = world_t_frame[1:,1:]
    # plot 
    ax.quiver(world_p_origin[0], world_p_origin[1], world_p_origin[2], world_r_origin[0,0], world_r_origin[1,0], world_r_origin[2,0], normalize=True, color="red")
    ax.quiver(world_p_origin[0], world_p_origin[1], world_p_origin[2], world_r_origin[0,1], world_r_origin[1,1], world_r_origin[2,1], normalize=True, color="green")
    ax.quiver(world_p_origin[0], world_p_origin[1], world_p_origin[2], world_r_origin[0,2], world_r_origin[1,2], world_r_origin[2,2], normalize=True, color="blue")

def visualize_dh_parameters(dh_parameters : list[DHParameters]) -> None:
    """
    We assume the list of dh_parameters describe a serial kinematic chain
    """
    fig = plt.figure()
    ax = plt.axes(projection='3d')
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')

    ax.set_xlim((-0.5,1.5))
    ax.set_ylim((-0.5,1.5))
    ax.set_zlim((-0.5,1.5))

    linki_t_linkj_list = [get_a_t_b(dh_parameter) for dh_parameter in  dh_parameters] # j = i+1

    print(linki_t_linkj_list)

    world_t_linki_list = [np.eye(4)]
    draw_frame_axes(world_t_linki_list[0], ax)

    for linki_t_linkj in linki_t_linkj_list:
        world_t_linkj = np.matmul(world_t_linki_list[-1], linki_t_linkj)
        print(world_t_linkj)
        draw_frame_axes(world_t_linkj, ax)
        world_t_linki_list.append(world_t_linkj)

    print(len(world_t_linki_list))

    plt.show()

# TODO: redo actual measurements
    
### Test script
dh_01 = DHParameters(
    d = 0.06985,
    theta = np.pi, # TODO: user input
    r = 0,
    alpha = 0,
)


dh_12 = DHParameters(
    d = 0,#0.017462,
    theta = np.pi/2.0, #0 TODO: user input
    r = 0.01111,#-0.01111, # TODO: confirm this is okay to make negative and isn't a mistake?
    alpha = np.pi/2.0,#np.pi/2.0
)


dh_23 = DHParameters(
    d = 0,
    theta = 0, # TODO: user input
    r = 0.08,
    alpha = 0
)


dh_34 = DHParameters(
    d = 0,
    theta = np.pi/2.0, # TODO: user input
    r = 0.08,
    alpha = 0
)
dh_45 = DHParameters(
    d = 0,
    theta = 0, # TODO: user input
    r = 0,
    alpha = np.pi/2.0
)

dh_56 = DHParameters(
    d = 0.09,# TODO: user input
    theta = 0, 
    r = 0,
    alpha = 0
)

learm_dh_parameters = [dh_01, dh_12,dh_23,dh_34,dh_45,dh_56]

print(learm_dh_parameters)

visualize_dh_parameters(learm_dh_parameters[:6])