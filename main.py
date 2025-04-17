import sys
sys.path.append('build/lab4/Release')

import numpy as np
import argparse

from touch import Touch

from igmr_robotics_toolkit.viewer.core import create_simple_viewer, Camera
from igmr_robotics_toolkit.viewer.widget import TransformWidget, MeshWidget, LineWidget, ImagePlaneWidget
from igmr_robotics_toolkit.util.xform import parse_xform
from igmr_robotics_toolkit.math import translate
from igmr_robotics_toolkit.data import DATA_ROOT

sphere_path = DATA_ROOT / 'sphere.obj'
cube_path = DATA_ROOT / 'cube.obj'


base2stylus_trans = parse_xform("rot(90d, 0, 0)") # 4x4
# base2stylus_trans = np.array([[1, 0, 0, 0],
#                               [0, 0, -1, 0],
#                               [0, 1, 0, 0],
#                               [0, 0, 0, 1]])
camera_trans = parse_xform("rot(-20d, 10d, -20d)")

def get_haptic_pose_m():
    '''Return 4x4 for stylus pose'''
    mtx_r = np.asanyarray(dev.get_transform()).reshape((4, 4))
    mtx_r[:3, 3] /= 1000
    mtx_r = base2stylus_trans @ mtx_r
    return  mtx_r

if __name__ == "__main__":

    # Get mode from user input
    parser = argparse.ArgumentParser(description="WN25 MedRob Lab4")
    parser.add_argument(
        '--mode',
        type=str,
        required=True,
        choices=["spring", "sphere", "cube", "viscosity", "friction", "line", "curve", "extra"],
        help="Specify haptic mode (must be one of: 'spring', 'sphere', 'cube', 'viscosity', 'friction', 'line', 'curve')"
    )
    args = parser.parse_args()

    mode_str = args.mode
    mode_list = ["spring", "sphere", "cube", "viscosity", "friction", "line", "curve", "extra"]
    mode_idx = mode_list.index(mode_str)
    print(f"Current haptic mode: {mode_str}")

    #Initialize device and set mode
    dev = Touch()
    dev.init_device()
    dev.set_mode(mode_idx+1)
    print("Started")

    # Start igmr
    window, root = create_simple_viewer(title="WN25 MedRob Lab4", orthographic=False)
    window._camera_state = Camera(translate(-0.175, -0.05, 0.2), camera_trans, 0.4)
    window._update_camera()

    stylus = TransformWidget(parent=root, track=get_haptic_pose_m, label="stylus", scale=0.1)
    stylus._label_text_node.set_glyph_scale(0.002)
    stylus._label_text_node.set_shadow(0.0001, 0.0001)

    base = TransformWidget(parent=root, label="base", scale=0.1)
    base._label_text_node.set_glyph_scale(0.002)
    base._label_text_node.set_shadow(0.0001, 0.0001)

    # Visualizer for each mode
    if mode_str == "spring":
        center = MeshWidget(path=cube_path, parent=root, scale=0.002)

    elif mode_str == "sphere":
        sphere = MeshWidget(path=sphere_path, parent=root, scale=0.05)
        sphere.tint = (1, 0, 0)
        sphere.opacity = 0.9

    elif mode_str == "cube":
        cube = MeshWidget(path=cube_path, parent=root, scale=0.05)
        cube.tint = (0, 1, 1)
        cube.opacity = 0.3

    elif mode_str == "viscosity":
        cube = MeshWidget(path=cube_path, parent=root, scale=0.05)
        cube.tint = (0, 1, 1)
        cube.opacity = 0.3

    elif mode_str == "friction":
        friction_rgb = 80* np.ones((90, 90, 3), dtype=np.uint8)
        friction_rgb[:, 30:60, :] = 40* np.ones((90, 30, 3), dtype=np.uint8)
        friction_rgb[:, 60:, :] = 20* np.ones((90, 30, 3), dtype=np.uint8)
        friction_plane = ImagePlaneWidget(parent=root, offset=(-0.045,-0.045), size=(0.09,0.09))
        friction_plane.load(image=friction_rgb)

    elif mode_str == "line":
        direction_control = 0.1 * np.array([1, 0.5, 0.2])
        direction_vis = base2stylus_trans[:3,:3] @ direction_control
        points = np.vstack((direction_vis, -direction_vis))

        line = LineWidget(parent=root)
        line.load(points)

    elif mode_str == "curve":
        t = np.linspace(-np.pi, np.pi, 100)

        z = 0.03 * t / (np.pi)
        x = 0.03*np.cos(2*t)
        y = 0.03*np.sin(2*t)

        points = np.column_stack((x, y, z))
        points = (base2stylus_trans[:3,:3] @ points.T).T

        line = LineWidget(parent=root)
        line.load(points)

        cube = MeshWidget(path=cube_path, parent=root, scale=0.08)
        cube.tint = (0, 1, 1)
        cube.opacity = 0.3

    elif mode_str == "extra":
        from pathlib import Path

       # funnel_path = Path("C:/Users/FRBGuest/Desktop/Lab4/medrob-lab4-main/funnels/Funnel.obj")
       # funnel_path = Path("C:/Users/FRBGuest/Desktop/Lab4/medrob-lab4-main/funnels/UM3E_Funnel 5.obj")
        funnel_path = Path("C:/Users/FRBGuest/Desktop/Lab4/medrob-lab4-main/funnels/UM3E_Funnel 7.obj")
        #funnel_path = cube_path

        print("Model Path:", funnel_path)



        if funnel_path.exists():
            funnel = MeshWidget(path=funnel_path, parent=root, scale=0.0005)
            funnel.tint = (1, 0, 0)
            funnel.opacity = 0.9
        else:
           print(f"File not found: {funnel_path}")


    window.run()
