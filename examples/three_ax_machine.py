import os
from urdf_scene_nicegui import UrdfScene
from nicegui import ui

script_dir = os.path.dirname(os.path.realpath(__file__))
scene = UrdfScene(os.path.join(script_dir, "urdf/three_ax_machine/osaca2.urdf"))
scene.show(material="#888", scale_stls=1)

# show axes control sliders as example how to manipulate axes
with ui.row().classes("w-full"):
    for joint in scene.get_joint_names():
        with ui.row().classes("w-1/5"):
            ui.label(joint + ":")
            x_min = scene.joint_pos_limits[joint]["min"]
            x_max = scene.joint_pos_limits[joint]["max"]
            n_slider_steps = 100
            ui.slider(
                min=x_min, max=x_max, step=(x_max-x_min)/n_slider_steps, value=(x_max+x_min)/2
            ).on(
                "update:model-value",
                lambda e, scene=scene, joint=joint: scene.set_axis_value(joint, e.args),
                throttle=0.1,
            ).props(
                "label-always"
            )
            
scene.scene.move_camera(x=0.2, y=0.8, z=1.5, look_at_x=0.5, look_at_y=0.2, look_at_z=1.2)

## overwrite all axes values to zero:
# num_axes = scene.get_num_axes()
# scene.set_axis_values([0]*num_axes)


ui.run(favicon="🤖")
