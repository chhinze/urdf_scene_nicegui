import re
from urchin import URDF
import os
from pathlib import Path
import tempfile
from typing import List, Tuple
import numpy as np
from scipy.spatial.transform import Rotation as R
from nicegui import ui, app


class UrdfScene:
    """ Loads a URDF file as nicegui.ui.scene (3d scene). 

    The constructor (`__init__`) is loading the URDF, method `show()` is building the scene (separating the object from its presentation).

    Axis values can be animated via methods 
    - `set_axis_value("joint_name", val)` (for one joint, call `get_joint_names()` for list of joint names)
    - `set_axis_values(list_of_values)` to set new values for all joints at once
    - property (dict) `joint_pos_values` holds for each joint name the position limits as fields `min` and `max` (None, if no limit is defined)
    
    Rules for the URDF:
    - meshes must reside in directory meshes/ located at directory of .urdf file or its parent directory
    - only STL meshes are supported (limitation of nicegui scene)
    - only supported joint types with 0-1 DOF ("fixed", "prismatic", "revolute")

    Example code:
    
    ```py   
    from nicegui import ui
    from urdf_scene_nicegui import UrdfScene

    scene = UrdfScene("path/to/urdf/my.urdf")
    scene.show()
    ui.run()
    ```

    more advanced: see at bottom of file
    """

    def __init__(self, urdf_path: str):
        """load a URDF file to construct a nicegui scene"""
        self.urdf_model = UrdfScene._load_urdf(
            urdf_path,
        )
        self.meshes_url = f"/meshes/{self.urdf_model.name}"
        self.joint_names = self.urdf_model.actuated_joint_names
        meshes_link = os.path.join(os.path.dirname(urdf_path), "meshes")
        if not os.path.exists(meshes_link):
            meshes_link = os.path.join(os.path.dirname(urdf_path), "../meshes")
        if not os.path.exists(meshes_link):
            raise NotADirectoryError(meshes_link)

        app.add_static_files(self.meshes_url, meshes_link)

        self.joint_groups = dict()
        self.joint_pos_limits = dict()
        self.joint_trafos = dict()
        self.scene = None

    def show(self, scale_stls=1, material=None):
        """plot a nicegui 3D scene from loaded URDF.

        Inputs:
        `scale_stls` (default:1) scales all STL files. E.g. pass 1e-1 if STLs are designed in mm
        `material` [default: None] color for the whole URDF (overrides mesh colors in STLs if defined), e.g. "#FFF" makes everything white
        """
        with ui.scene(grid=False).classes("w-full h-[66vh]") as self.scene:
            self._plot_stls(
                self.urdf_model.base_link, scale=scale_stls, material=material
            )
            next_joints = self._get_next_joints(
                self.urdf_model, self.urdf_model.base_link
            )
            for joint in next_joints:
                self._recursively_add_subtree(
                    self.urdf_model, joint, scale_stls=scale_stls, material=material
                )

    def is_initialized(self) -> bool:
        return self.scene is not None

    def set_axis_value(self, joint_name: str, val: float) -> None:
        t, r = self.joint_trafos[joint_name](val)
        self.joint_groups[joint_name].move(*t).rotate(*r)
        # ui.notify(f"{joint_name} moved to [{self.joint_groups[joint_name].x}, {self.joint_groups[joint_name].y}, {self.joint_groups[joint_name].z}]")

    def set_axis_values(self, val: List | np.array) -> None:
        """Set all axes values to new values by passing an array or list.

        order of values in `val` is determined by `self.joint_names` list."""
        for joint_name, val in zip(self.joint_names, list(val)):
            joint_TF = self.joint_trafos[joint_name]
            joint_i = self.joint_groups[joint_name]
            t, r = joint_TF(val)
            joint_i.move(*t).rotate(*r)

    def get_joint_names(self) -> List[str]:
        return self.joint_groups.keys()

    def _get_next_joints(
        self,
        urdf,
        link_obj,
    ):
        next_joints = [j for j in urdf.joints if j.parent == link_obj.name]
        return next_joints

    def _recursively_add_subtree(self, urdf, joint, scale_stls=1, material=None):
        t, r = UrdfScene.get_transl_and_rpy(joint.origin)
        with ui.scene.group().move(*t).rotate(*r):
            with ui.scene.group() as joint_trafo:
                if joint.joint_type != "fixed":
                    self.joint_groups[joint.name] = joint_trafo
                    # ui.notify(f"{joint.name} ({joint.joint_type}): {joint.axis}")
                    if joint.joint_type == "prismatic":
                        self.joint_trafos[joint.name] = (
                            lambda q, axis=joint.axis: UrdfScene.transl_joint(axis, q)
                        )
                    elif joint.joint_type == "revolute":
                        self.joint_trafos[joint.name] = (
                            lambda q, axis=joint.axis: UrdfScene.rot_joint(axis, q)
                        )
                    else:
                        raise NotImplementedError(
                            f"Cannot process joint type {joint.joint_type}. Supported joint types are 'fixed', 'prismatic', 'revolute'."
                        )
                    
                    self.joint_pos_limits[joint.name] = {'min': joint.limit.lower, 'max': joint.limit.upper}   

                child_link = [l for l in urdf.links if l.name == joint.child]
                if child_link:
                    child_link = child_link[0]
                    self._plot_stls(child_link, scale=scale_stls, material=material)

                    if child_link not in urdf.end_links:
                        joints = self._get_next_joints(urdf, child_link)
                        for joint in joints:
                            self._recursively_add_subtree(
                                urdf, joint, scale_stls=scale_stls, material=material
                            )
                    else:  # child is end link
                        self._draw_scene_cos(scale=0.05)

    def _plot_stls(self, link, scale=1, material=None):
        for visual in link.visuals:
            if material is not None:
                ui.scene.stl(self._stl_to_url(visual.geometry.geometry.filename)).scale(
                    scale
                ).material(material)
            else:
                ui.scene.stl(self._stl_to_url(visual.geometry.geometry.filename)).scale(
                    scale
                )

    def _stl_to_url(self, stl_path: str) -> str:
        return os.path.join(self.meshes_url, os.path.basename(stl_path))

    def _draw_scene_cos(self, scale=0.3, translate=np.array([0, 0, 0])):
        self.scene.line(
            translate.tolist(), (np.array([scale, 0, 0]) + translate).tolist()
        ).material("#ff0000")
        self.scene.line(
            translate.tolist(), (np.array([0, scale, 0]) + translate).tolist()
        ).material("#00ff00")
        self.scene.line(
            translate.tolist(), (np.array([0, 0, scale]) + translate).tolist()
        ).material("#0000ff")

    @classmethod
    def _load_urdf(
        cls, urdf_path: str, meshes_path: str = "", lazy: bool = True
    ) -> URDF:
        """loads a urdf file to memory"""

        def extract_package_name(urdf_path: str) -> str:
            with open(urdf_path, "r") as file:
                urdf_contents = file.read()
                package_name = re.search(r"(?<=package://)\w+", urdf_contents).group(0)

            return package_name

        urdf_file_path = Path(urdf_path)
        package_name = extract_package_name(urdf_path)

        # replace in temporary file: package://... with absolute path
        with tempfile.TemporaryDirectory() as tmpdirname:
            tmp_file_path = Path(tmpdirname) / urdf_file_path.name

            with open(urdf_path, "r") as fin:
                with open(tmp_file_path, "w") as fout:
                    for line in fin:
                        fout.write(
                            line.replace(
                                "package://" + package_name,
                                str(urdf_file_path.resolve().parent),
                            )
                        )

            return URDF.load(tmp_file_path, lazy_load_meshes=lazy)

    @classmethod
    def get_transl_and_rpy(cls, mat):
        """returns translation vector and rpy vector from 4x4 homogeneous transformation"""
        trans = mat[:3, 3]
        rpy = R.from_matrix(mat[:3, :3]).as_euler("xyz", degrees=False)
        return trans, rpy

    @classmethod
    def rot_joint(
        cls, axis: np.ndarray, rot_rad: float
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Get the transformation for a rotatory joint rotating around `axis` with value `rot_rad` [rad].

        returns: translation (3x1) = [0,0,0], rpy rotation (3x1)"""
        norm_axis = axis / np.linalg.norm(axis)
        # rot_matrix = R.from_rotvec(rot_rad*norm_axis).as_matrix()
        # T = np.eye(4)
        # T[:3,:3] = rot_matrix
        rpy = R.from_rotvec(rot_rad * norm_axis).as_euler("xyz", degrees=False)
        t = np.zeros_like(rpy)
        return t, rpy

    @classmethod
    def transl_joint(
        cls, axis: np.array, transl: float
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Get the transformation for a translational joint along `axis` with value `transl` [m].

        returns: translation (3x1), rpy rotation (3x1) = [0,0,0]"""
        norm_axis = axis / np.linalg.norm(axis)
        t = transl * norm_axis
        rpy = np.zeros_like(t)
        # T = np.eye(4)
        # T[:3,3] = T
        return t, rpy
