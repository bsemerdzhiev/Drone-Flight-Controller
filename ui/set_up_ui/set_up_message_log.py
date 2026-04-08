import dearpygui.dearpygui as dpg

import util.data as stored_data


def set_up_message_log():
    with dpg.tab(label="All Messages Log"):
        with dpg.child_window(height=-1, border=False):
            dpg.add_text("Message Log", color=[255, 255, 100])
            with dpg.table(
                tag="msg_table",
                header_row=True,
                borders_innerH=True,
                borders_outerH=True,
                borders_innerV=True,
                borders_outerV=True,
                scrollY=True,
                freeze_rows=1,
                height=560,
            ):
                dpg.add_table_column(
                    label="Time", width_fixed=True, init_width_or_weight=70
                )
                dpg.add_table_column(
                    label="Direction", width_fixed=True, init_width_or_weight=90
                )
                dpg.add_table_column(label="Message")
