"""
I highly recommend checking out "A Note on Denavit-Hartenberg Notation in Robotics by Harvey Lipkin (2005)
This codebase was based on how the proximal variant was described in it. I am highly greatful for it as a resource.
"""
import numpy.typing as npt
import matplotlib.pyplot as plt

from learm.kinematics import get_dh_parameters, get_link0_t_linki, DHParameters, LEARM_JOINT_OFFSETS
from learm.visualization import init_matplotlib_fig, draw_dh_parameters, add_sliders

# Define a closure to capture the correct index for each slider
def create_update_function(idx):
    def update_function(x):
        update(x, idx)
    return update_function

# The function to be called anytime a slider's value changes
def update(val,updating_idx):
    global viz_axes_list,viz_link_list, learm_dh_parameters, world_t_linki_list
    artists = viz_axes_list + viz_link_list
    for artist in artists:
        artist.remove()
    # update existing learn_dh_parameters
    # most joints are actuated via theta, only last is by d.
    print(f"val: {val}, idx: {updating_idx}")
    if updating_idx != 5:
        learm_dh_parameters[updating_idx].theta = val
    else:
        learm_dh_parameters[updating_idx].d = val
    world_t_linki_list = get_link0_t_linki(learm_dh_parameters)
    viz_axes_list,viz_link_list = draw_dh_parameters(world_t_linki_list,ax)
    fig.canvas.draw_idle()

learm_dh_parameters : list[DHParameters] = get_dh_parameters()
world_t_linki_list : npt.NDArray = get_link0_t_linki(learm_dh_parameters)

fig, ax = init_matplotlib_fig()
viz_axes_list, viz_link_list = draw_dh_parameters(world_t_linki_list, ax)
joint_slider_list = add_sliders(fig, ax)

update_function_list = [create_update_function(idx) for idx in range(len(joint_slider_list))]

for jointi_slider, update_function in zip(joint_slider_list, update_function_list):
    jointi_slider.on_changed(update_function)

plt.show()