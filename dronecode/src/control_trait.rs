use my_hdlc::command::FSMState;

pub trait FSMControl {
    fn run_control_loop(&self, transceiver: &mut my_hdlc::HdlcTransceiver);
    // fn run_safe_mode_cl(& self);
    fn step(&self, next_state: FSMState) -> &dyn FSMControl;
}
