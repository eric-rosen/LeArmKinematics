"""
I highly recommend checking out "A Note on Denavit-Hartenberg Notation in Robotics by Harvey Lipkin (2005)
This codebase was based on how the proximal variant was described in it. I am highly greatful for it as a resource.
"""
from dataclasses import dataclass
import numpy as np
import numpy.typing as npt
import math
import matplotlib.pyplot as plt
from typing import List
from matplotlib.widgets import Slider

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

def draw_frame_axes(world_t_frame : npt.NDArray, ax) -> None:
    world_p_origin = world_t_frame[1:,0]
    world_r_origin = world_t_frame[1:,1:] / 30.0
    # plot axes
    ax1 = ax.quiver(world_p_origin[0], world_p_origin[1], world_p_origin[2], world_r_origin[0,0], world_r_origin[1,0], world_r_origin[2,0], color="red")
    ax2 = ax.quiver(world_p_origin[0], world_p_origin[1], world_p_origin[2], world_r_origin[0,1], world_r_origin[1,1], world_r_origin[2,1],  color="green")
    ax3 = ax.quiver(world_p_origin[0], world_p_origin[1], world_p_origin[2], world_r_origin[0,2], world_r_origin[1,2], world_r_origin[2,2],  color="blue")
    return([ax1,ax2,ax3])

def draw_link_axes(world_t_frame1 : npt.NDArray, world_t_frame2 : npt.NDArray, ax) -> None:
    world_p_frame1 = world_t_frame1[1:,0]
    world_p_frame2 = world_t_frame2[1:,0]
    # quiver wants frame2 in frame1
    frame1_p_frame2 = world_p_frame2 - world_p_frame1
    # plot link
    link_quiver = ax.quiver(world_p_frame1[0], world_p_frame1[1], world_p_frame1[2], frame1_p_frame2[0], frame1_p_frame2[1], frame1_p_frame2[2], color="black")
    return(link_quiver)
def get_link0_t_linki(dh_parameters : list[DHParameters]) -> List[npt.NDArray]:
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

def init_matplotlib_fig():
    """
    3d figure initialization for LeArm
    """
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')

    ax.set_xlim((-0.15,.15))
    ax.set_ylim((-0.15,.15))
    ax.set_zlim((-0.05,.25))
    return (fig,ax)



def draw_dh_parameters(world_t_linki_list : list[npt.NDArray],ax) -> None:
    """
    We assume the list of dh_parameters describe a serial kinematic chain
    """
    # Draw the frames
    viz_axes_list = []
    for link0_t_linkj in world_t_linki_list:
        viz_axes_list += draw_frame_axes(link0_t_linkj, ax)
    
    # Draw the links
    viz_link_list = []
    for linkj in range(len(world_t_linki_list)-1):
        # draw two most recent links
        viz_link_list.append(draw_link_axes(world_t_linki_list[linkj], world_t_linki_list[linkj+1], ax))

    return(viz_axes_list,viz_link_list)

# TODO: redo actual measurements
def get_dh_parameters(q_offset_1 : float = 0 ,
                      q_offset_2 : float = 0 ,
                      q_offset_3 : float = 0 ,
                      q_offset_4 : float = 0 ,
                      q_offset_5 : float = 0 ,
                      q_offset_6 : float = 0 ,
                      ):
    ### Test script
    dh_01 = DHParameters(
        d = 0.06985,
        theta = np.pi - q_offset_1, # TODO: user input
        r = 0,
        alpha = 0,
    )


    dh_12 = DHParameters(
        d = 0,
        theta = np.pi/2.0 + q_offset_2, #0 TODO: user input
        r = 0.01111,
        alpha = np.pi/2.0,
    )


    dh_23 = DHParameters(
        d = 0,
        theta = 0 + q_offset_3, # TODO: user input
        r = 0.08,
        alpha = 0
    )


    dh_34 = DHParameters(
        d = 0,
        theta = np.pi/2.0 + q_offset_4, # TODO: user input
        r = 0.08,
        alpha = 0
    )
    dh_45 = DHParameters(
        d = 0,
        theta = 0 + q_offset_5, # TODO: user input
        r = 0,
        alpha = np.pi/2.0
    )

    dh_56 = DHParameters(
        d = 0.09 + q_offset_6,# TODO: user input
        theta = 0, 
        r = 0,
        alpha = 0
    )

    learm_dh_parameters = [dh_01, dh_12,dh_23,dh_34,dh_45,dh_56]
    return(learm_dh_parameters)

def add_sliders(fix, ax):
    # adjust the main plot to make room for the sliders
    fig.subplots_adjust(left=0.25, bottom=0.25)

    joint_slider_list = []
    # revolute joints
    for jointi in range(5):
        # Make a horizontal slider to control joints
        axfreq = fig.add_axes([0.25, 0 + (0.05*jointi), 0.65, 0.03])
        joint_slider_list.append(
        Slider(
            ax=axfreq,
            label=f'joint {jointi}',
            valmin=-np.pi/2.0,
            valmax=np.pi/2.0,
            valinit=0,
        ))
    # last joint is the gripper which we model as translating rather than rotationally
    # it has different bounds than the others
    axfreq = fig.add_axes([0.25, 0 + (0.05*5), 0.65, 0.03])
    joint_slider_list.append(
        Slider(
            ax=axfreq,
            label=f'joint 5',
            valmin=0,
            valmax=np.pi,
            valinit=0,
        ))

    return joint_slider_list


# The function to be called anytime a slider's value changes
def update(val):
    global viz_axes_list,viz_link_list, learm_dh_parameters, world_t_linki_list
    artists = viz_axes_list + viz_link_list
    for artist in artists:
        artist.remove()
    learm_dh_parameters = get_dh_parameters(q_offset_1=val)
    world_t_linki_list = get_link0_t_linki(learm_dh_parameters)
    viz_axes_list,viz_link_list = draw_dh_parameters(world_t_linki_list,ax)
    fig.canvas.draw_idle()

learm_dh_parameters : list[DHParameters]= get_dh_parameters()
world_t_linki_list : npt.NDArray = get_link0_t_linki(learm_dh_parameters)

fig,ax = init_matplotlib_fig()
viz_axes_list,viz_link_list = draw_dh_parameters(world_t_linki_list,ax)
joint_slider_list = add_sliders(fig,ax)
for jointi, jointi_slider in enumerate(joint_slider_list):
    jointi_slider.on_changed(update)
plt.show()