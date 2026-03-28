from time import sleep
import numpy as np
import pyqtgraph.opengl as gl
from pyqtgraph.Qt import QtWidgets, QtCore
from scipy.spatial.transform import Rotation


def make_drone_view():
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    view = gl.GLViewWidget()
    view.setWindowTitle("Drone 3D View")
    view.setCameraPosition(distance=3)
    view.show()

    # grid
    grid = gl.GLGridItem()
    view.addItem(grid)

    # drone arms: two crossing lines (X shape)
    arm_length = 0.5
    arm_verts = np.array(
        [
            [-arm_length, -arm_length, 0],
            [arm_length, arm_length, 0],
            [-arm_length, arm_length, 0],
            [arm_length, -arm_length, 0],
        ]
    )
    arms = gl.GLLinePlotItem(
        pos=np.array([arm_verts[0], arm_verts[1]]),
        color=(1, 1, 1, 1),
        width=2,
        antialias=True,
    )
    arms2 = gl.GLLinePlotItem(
        pos=np.array([arm_verts[2], arm_verts[3]]),
        color=(1, 1, 1, 1),
        width=2,
        antialias=True,
    )
    view.addItem(arms)
    view.addItem(arms2)

    # propeller positions (match arm tips)
    prop_positions = np.array(
        [
            [-arm_length, -arm_length, 0],  # M0
            [arm_length, arm_length, 0],  # M1
            [-arm_length, arm_length, 0],  # M2
            [arm_length, -arm_length, 0],  # M3
        ]
    )

    # one scatter item per propeller so we can color them independently
    prop_dots = []
    for pos in prop_positions:
        dot = gl.GLScatterPlotItem(
            pos=pos.reshape(1, 3),
            size=15,
            color=(0.0, 1.0, 0.0, 1.0),  # start green
            pxMode=True,
        )
        view.addItem(dot)
        prop_dots.append(dot)

    # body center dot
    center = gl.GLScatterPlotItem(
        pos=np.array([[0, 0, 0]]), size=10, color=(1, 1, 1, 1)
    )
    view.addItem(center)

    motor_labels = []
    label_offsets = [
        [-arm_length - 0.1, -arm_length - 0.1, 0.05],  # M0
        [arm_length + 0.1, arm_length + 0.1, 0.05],  # M1
        [-arm_length - 0.1, arm_length + 0.1, 0.05],  # M2
        [arm_length + 0.1, -arm_length - 0.1, 0.05],  # M3
    ]
    for i, offset in enumerate(label_offsets):
        label = gl.GLTextItem(
            pos=np.array(offset), text=f"M{i}: 0", color=(255, 255, 255, 255)
        )
        view.addItem(label)
        motor_labels.append(label)

    return view, arms, arms2, prop_dots, arm_verts, motor_labels


def rpm_to_color(rpm: int, max_rpm: int = 800):
    t = max(0.0, min(1.0, rpm / max_rpm))
    # green -> yellow -> red
    if t < 0.5:
        return (t * 2, 1.0, 0.0, 1.0)
    else:
        return (1.0, 1.0 - (t - 0.5) * 2, 0.0, 1.0)


def update_drone_view(
    arms,
    arms2,
    prop_dots,
    arm_verts,
    motor_labels,
    yaw,
    pitch,
    roll,
    motor_values,
):
    r = Rotation.from_euler("ZYX", [yaw, pitch, roll], degrees=True)
    rotated = r.apply(arm_verts)

    arms.setData(pos=np.array([rotated[0], rotated[1]]))
    arms2.setData(pos=np.array([rotated[2], rotated[3]]))

    for i, dot in enumerate(prop_dots):
        dot.setData(pos=rotated[i].reshape(1, 3), color=rpm_to_color(motor_values[i]))
        motor_labels[i].setData(
            pos=rotated[i] + np.array([0.05, 0.05, 0.05]),
            text=f"M{i}: {motor_values[i]}",
        )
