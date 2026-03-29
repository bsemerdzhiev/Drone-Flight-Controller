import threading
import sys
import dearpygui.dearpygui as dpg
from pyqtgraph.Qt import QtWidgets, QtCore
from read_data import serial_reader
from visualize import make_drone_view, update_drone_view
from update_gui import update_step
from set_up import set_up_gui
import data as stored_data


def run_dpg():
    set_up_gui()
    dpg.set_frame_callback(1, update_step)
    dpg.create_viewport(title="Drone UI", width=1920, height=1080)
    dpg.setup_dearpygui()
    dpg.show_viewport()
    dpg.start_dearpygui()
    dpg.destroy_context()


def qt_update(
    arms,
    arms2,
    prop_dots,
    arm_verts,
    motor_labels,
):
    if len(stored_data.live_data.time_data) == 0:
        return
    update_drone_view(
        arms,
        arms2,
        prop_dots,
        arm_verts,
        motor_labels,
        stored_data.live_data.yaw_kalman[-1],
        stored_data.live_data.pitch_kalman[-1],
        stored_data.live_data.roll_kalman[-1],
        stored_data.live_data.motors[-1],
    )


if __name__ == "__main__":
    qt_app = QtWidgets.QApplication(sys.argv)
    view, arms, arms2, prop_dots, arm_verts, motor_labels = make_drone_view()

    threading.Thread(target=serial_reader, daemon=True).start()
    threading.Thread(target=run_dpg, daemon=True).start()

    timer = QtCore.QTimer()
    timer.timeout.connect(
        lambda: qt_update(arms, arms2, prop_dots, arm_verts, motor_labels)
    )
    timer.start(50)

    qt_app.exec()
