use serde::{Deserialize, Serialize};

#[derive(Serialize, Deserialize, PartialEq)]
#[repr(u8)]
pub enum CommandLiterals {
    Left,
    Right,
    Top,
    Bottom,
    GetSteps,
    StepCount,
    Path,
    Reset,
}

#[derive(Serialize, Deserialize, PartialEq)]
pub struct Command {
    to_send: CommandLiterals,
    payload: u8,
}

impl Command {
    pub fn new(to_send: CommandLiterals, payload: u8) -> Self {
        Self { to_send, payload }
    }

    pub fn get_command(&mut self) -> &CommandLiterals {
        &self.to_send
    }

    pub fn get_payload(&mut self) -> u8 {
        self.payload
    }
}
