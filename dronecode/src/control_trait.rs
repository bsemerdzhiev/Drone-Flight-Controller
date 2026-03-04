use my_hdlc::command::{Command, FSMState};

pub trait FSMControl {
    fn run_control_loop(
        &self,
        command: Option<Command>,
        transceiver: &mut my_hdlc::HdlcTransceiver,
    );
    // fn run_safe_mode_cl(& self);
    fn step(&self, next_state: FSMState) -> &dyn FSMControl;
}
