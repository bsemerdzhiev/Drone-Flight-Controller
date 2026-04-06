import threading
import sys
import dearpygui.dearpygui as dpg
from pyqtgraph.Qt import QtWidgets, QtCore
from read_data.read_data import serial_reader
from drone_visualization.visualize import make_drone_view, update_drone_view
from update_ui.update_gui import update_step
from set_up_ui.set_up import set_up_gui
import util.data as stored_data


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
    with stored_data.message_log_lock:
        if len(stored_data.live_data.time_data) == 0:
            return

        provide_data = [
            stored_data.live_data.yaw_data[-1],
            stored_data.live_data.pitch_data[-1],
            stored_data.live_data.roll_data[-1],
        ]

        if stored_data.chosen_sensors:
            provide_data = [
                0,
                stored_data.live_data.pitch_kalman[-1],
                stored_data.live_data.roll_kalman[-1],
            ]

        update_drone_view(
            arms,
            arms2,
            prop_dots,
            arm_verts,
            motor_labels,
            provide_data[0],
            provide_data[1],
            provide_data[2],
            [
                stored_data.live_data.motors[0][-1],
                stored_data.live_data.motors[1][-1],
                stored_data.live_data.motors[2][-1],
                stored_data.live_data.motors[3][-1],
            ],
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
    timer.start(100)

    qt_app.exec()
