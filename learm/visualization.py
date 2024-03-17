"""
Holds all utility functions relevant to visualizing LeArm.

Most important is InteractiveVisualizer, which is a class
that on construction generates an interactive
3D matplotlib rendering of the LeArm.
"""
import numpy.typing as npt
import matplotlib.pyplot as plt
import numpy as np
from learm.kinematics import get_dh_parameters, get_link0_t_linki, DHParameters, LEARM_JOINT_OFFSETS, LEARN_JOINT_MIN_MAX
from matplotlib.widgets import Slider

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

def add_sliders(fig,
                ax,
                q_0 : float = LEARM_JOINT_OFFSETS[0],
                q_1  : float = LEARM_JOINT_OFFSETS[1] ,
                q_2  : float = LEARM_JOINT_OFFSETS[2] ,
                q_3  : float = LEARM_JOINT_OFFSETS[3] ,
                q_4  : float = LEARM_JOINT_OFFSETS[4] ,
                q_5  : float = LEARM_JOINT_OFFSETS[5] ,):
    # adjust the main plot to make room for the sliders
    fig.subplots_adjust(left=0.25, bottom=0.25)

    joint_slider_list = []
    # joints
    q_list = [q_0,q_1,q_2,q_3,q_4,q_5]
    for jointi, (jointi_min_max, q_val) in enumerate(zip(LEARN_JOINT_MIN_MAX,q_list)):
        # Make a horizontal slider to control joints
        axfreq = fig.add_axes([0.25, 0 + (0.05*jointi), 0.65, 0.03])
        joint_slider_list.append(
        Slider(
            ax=axfreq,
            label=f'joint {jointi}',
            valmin=jointi_min_max[0],
            valmax=jointi_min_max[1],
            valinit=q_val,
        ))

    return joint_slider_list


class InteractiveVisualizer():
    def __init__(self,
                 q_0 : float = LEARM_JOINT_OFFSETS[0],
                 q_1  : float = LEARM_JOINT_OFFSETS[1] ,
                 q_2  : float = LEARM_JOINT_OFFSETS[2] ,
                 q_3  : float = LEARM_JOINT_OFFSETS[3] ,
                 q_4  : float = LEARM_JOINT_OFFSETS[4] ,
                 q_5  : float = LEARM_JOINT_OFFSETS[5] ):
        # starts in home configuration
        self.learm_dh_parameters : list[DHParameters] = get_dh_parameters(q_0=q_0,
                                                                          q_1=q_1,
                                                                          q_2=q_2,
                                                                          q_3=q_3,
                                                                          q_4=q_4,
                                                                          q_5=q_5)
        world_t_linki_list : npt.NDArray = get_link0_t_linki(self.learm_dh_parameters)

        self.fig, self.ax = init_matplotlib_fig()
        self.viz_axes_list, self.viz_link_list = draw_dh_parameters(world_t_linki_list, self.ax)

        joint_slider_list = add_sliders(self.fig, 
                                        self.ax,q_0=q_0,
                                        q_1=q_1,
                                        q_2=q_2,
                                        q_3=q_3,
                                        q_4=q_4,
                                        q_5=q_5)

        update_function_list = [self.create_update_function(idx) for idx in range(len(joint_slider_list))]

        for jointi_slider, update_function in zip(joint_slider_list, update_function_list):
            jointi_slider.on_changed(update_function)

        plt.show()

    # Define a closure to capture the correct index for each slider
    def create_update_function(self, idx):
        def update_function(x):
            self.update(x, idx)
        return update_function

    # The function to be called anytime a slider's value changes
    def update(self, val,updating_idx):
        artists = self.viz_axes_list + self.viz_link_list
        for artist in artists:
            artist.remove()
        # update existing learn_dh_parameters
        # most joints are actuated via theta, only last is by d.
        print(f"val: {val}, idx: {updating_idx}")
        if updating_idx != 5:
            self.learm_dh_parameters[updating_idx].theta = val
        else:
            self.learm_dh_parameters[updating_idx].d = val

        world_t_linki_list = get_link0_t_linki(self.learm_dh_parameters)
        self.viz_axes_list, self.viz_link_list = draw_dh_parameters(world_t_linki_list,self.ax)
        self.fig.canvas.draw_idle()


if __name__ == "__main__":
    InteractiveVisualizer()