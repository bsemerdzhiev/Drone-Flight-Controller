#![no_std]

#[cfg(test)]
extern crate std;

use crc::{CRC_32_ISCSI, Crc};
use serde::{Deserialize, Serialize};

/*
* A serialized DeviceCommand struct consists of 2 bytes + 1 byte for the CRC. Therefore, the MESSAGE_SIZE used for
* serializing the DeviceCommand struct is set to be 8 bytes. After escaping the control bytes inside the
* payload, and adding the two framing bytes, we can end up with at most (2*3 + 2)8 bytes. In total,
* we can fit 128/8 ~ 16 messages in the buffer.
*/
pub static BUFFER_SIZE: usize = 1 << 7;
pub static MESSAGE_SIZE: usize = 1 << 3;
// used as return size when serializing a structure
pub static STUFFED_MESSAGE_SIZE: usize = (1 << 3) * 3;

static FRAME_BOUNDARY: u8 = 0x7E;
static CTRL_ESCAPE: u8 = 0x7D;
static INV_BYTE: u8 = 1 << 5;

pub mod command;
pub mod telemetry_data;
pub struct HdlcTransceiver {
    /*
     * Buff keeps the bytes sent over uart and they are removed once a frame(two FRAME_BOUNDARY
     * bytes) have been received along with the bytes in between.
     * left_pointer points to the first byte in the fifo buffer.
     * right_pointer points to the last byte that was received.
     *
     * when either pointers reach BUFFER_SIZE, they are looped back to 0.
     */
    pub buff: [u8; BUFFER_SIZE],
    pub left_pointer: usize,
    pub right_pointer: usize,
    pub crc: Crc<u32>,

    /*
     * Since buff can be looped(start from 0 when overflowing),
     * helper_arr helps us look at it always starting from 0, thus
     * making the code easier to follow.
     */
    helper_arr: [u8; BUFFER_SIZE],

    /*
     * Is used to hold the bytes for the serialized structure along with the CRC computed for the
     * data.
     */
    removed_ctrl: [u8; MESSAGE_SIZE],
}

impl HdlcTransceiver {
    pub fn new() -> Self {
        Self {
            buff: [0; BUFFER_SIZE],
            left_pointer: 0,
            right_pointer: 0,
            helper_arr: [0; BUFFER_SIZE],
            removed_ctrl: [0; MESSAGE_SIZE],
            crc: Crc::<u32>::new(&CRC_32_ISCSI),
        }
    }

    /*
     * Serializes the structure to bytes, and stuff ctrl bytes and add CRC_8
     */
    pub fn write_structure<T>(
        &mut self,
        struc_to_serialize: &T,
    ) -> ([u8; STUFFED_MESSAGE_SIZE], usize)
    where
        T: Serialize,
    {
        let mut serialized_buff: [u8; MESSAGE_SIZE] = [0; MESSAGE_SIZE];
        postcard::to_slice(struc_to_serialize, &mut serialized_buff).unwrap();

        let checksum: u32 = self.crc.checksum(&serialized_buff);

        let mut stuffed_buff: [u8; STUFFED_MESSAGE_SIZE] = [0; STUFFED_MESSAGE_SIZE];

        serialized_buff[MESSAGE_SIZE - 4] = (checksum >> 24) as u8;
        serialized_buff[MESSAGE_SIZE - 3] = ((checksum >> 16) & 0xFF) as u8;
        serialized_buff[MESSAGE_SIZE - 2] = ((checksum >> 8) & 0xFF) as u8;
        serialized_buff[MESSAGE_SIZE - 1] = ((checksum) & 0xFF) as u8;

        stuffed_buff[0] = FRAME_BOUNDARY;
        let mut k: usize = 1;

        for i in 0..serialized_buff.len() {
            if serialized_buff[i] == CTRL_ESCAPE || serialized_buff[i] == FRAME_BOUNDARY {
                stuffed_buff[k] = CTRL_ESCAPE;
                k += 1;

                stuffed_buff[k] = serialized_buff[i] ^ INV_BYTE;
            } else {
                stuffed_buff[k] = serialized_buff[i];
            }
            k += 1;
        }
        stuffed_buff[k] = FRAME_BOUNDARY;
        k += 1;

        (stuffed_buff, k)
    }

    fn check_crc(&mut self) -> bool {
        let checksum: u32 = ((self.removed_ctrl[MESSAGE_SIZE - 4] as u32) << 24)
            | ((self.removed_ctrl[MESSAGE_SIZE - 3] as u32) << 16)
            | ((self.removed_ctrl[MESSAGE_SIZE - 2] as u32) << 8)
            | (self.removed_ctrl[MESSAGE_SIZE - 1] as u32);

        // set the last 4 bytes to zero
        for i in (MESSAGE_SIZE - 4)..MESSAGE_SIZE {
            self.removed_ctrl[i] = 0;
        }

        // recalculate the checksum again
        let new_checksum: u32 = self.crc.checksum(&self.removed_ctrl);

        checksum == new_checksum
    }

    /*
     * Transform the received bytes to a structure.
     */
    pub fn read_structure<'a, T>(&'a mut self) -> Option<T>
    where
        T: Deserialize<'a>,
    {
        /*
         * drop bytes until we find a frame boundary byte.
         * this way if a message gets corrupted and its frame boundary byte gets dropped/corrupted,
         * we will be able to process the next message by clearing out the junk
         */
        let mut removed_bytes = false;
        while self.left_pointer != self.right_pointer {
            if self.buff[self.left_pointer] == FRAME_BOUNDARY {
                // remove the frame boundary left from the corrupted message
                if removed_bytes {
                    self.left_pointer = (self.left_pointer + 1) % BUFFER_SIZE;
                }
                break;
            }
            removed_bytes = true;

            self.left_pointer = (self.left_pointer + 1) % BUFFER_SIZE;
        }

        let mut start_pnt: usize = self.left_pointer;
        let mut k: usize = 0;

        // find the next frame_boundary byte
        loop {
            if start_pnt == self.right_pointer {
                return None;
            }

            self.helper_arr[k] = self.buff[start_pnt];
            k += 1;

            // loop back to 0
            start_pnt = (start_pnt + 1) % BUFFER_SIZE;

            // if a frame boundary byte was seen and it was not the first one, break
            if self.helper_arr[k - 1] == FRAME_BOUNDARY && k != 1 {
                break;
            }
        }

        // since we have found another frame boundary byte, remove all those bytes from the fifo
        // and try to deserialize
        self.left_pointer = start_pnt;

        //helper_arr should countain one whole frame
        let mut escape_next_byte: bool = false;
        let mut new_ind: usize = 0;
        for i in 1..(k - 1) {
            if self.helper_arr[i] == CTRL_ESCAPE {
                // next byte is escaped
                escape_next_byte = true;
                continue;
            }

            // a right-edge frame boundary byte has been lost and we are trying to deserailize a
            // long message
            if new_ind >= MESSAGE_SIZE {
                return None;
            }

            if escape_next_byte {
                self.removed_ctrl[new_ind] = self.helper_arr[i] ^ INV_BYTE;
            } else {
                self.removed_ctrl[new_ind] = self.helper_arr[i];
            }
            escape_next_byte = false;
            new_ind += 1;
        }

        // check the message crc
        if new_ind != MESSAGE_SIZE || !self.check_crc() {
            return None;
        }

        // remove the last crc byte
        let deserialized_message =
            postcard::take_from_bytes::<T>(&self.removed_ctrl[0..(new_ind - 4)]);

        // check if the message can be deserialized
        if deserialized_message.is_err() {
            return None;
        }

        return Some(deserialized_message.unwrap().0);
    }

    pub fn add_byte(&mut self, byte_to_add: u8) {
        self.buff[self.right_pointer] = byte_to_add;

        self.right_pointer = (self.right_pointer + 1) % BUFFER_SIZE;

        // if we are starting to loop around the read index, start dropping the oldest bytes
        if self.right_pointer == self.left_pointer {
            self.left_pointer = (self.left_pointer + 1) % BUFFER_SIZE;
        }
    }
    pub fn fifo_is_empty(&mut self) -> bool {
        self.left_pointer == self.right_pointer
    }

    pub fn add_bytes(&mut self, bytes_to_add: &[u8]) {
        for byte in bytes_to_add {
            self.add_byte(*byte);
        }
    }
}

#[cfg(test)]
mod tests {
    use crate::DeviceCommand::FSMState;

    use super::*;

    use DeviceCommand::DeviceCommand;

    #[test]
    fn test_correctly_received() {
        let mut transceiver = HdlcTransceiver::new();

        let first_command: DeviceCommand = DeviceCommand::new(
            DeviceCommand::CommandType::ChangeMode,
            Some(FSMState::ManualMode),
        );

        let (arr, arr_size): ([u8; STUFFED_MESSAGE_SIZE], usize) =
            transceiver.write_structure::<DeviceCommand>(&first_command);

        for k in 0..arr_size {
            transceiver.add_byte(arr[k]);
        }

        let second_command: DeviceCommand = DeviceCommand::new(
            DeviceCommand::CommandType::ChangeMode,
            Some(FSMState::YawControl),
        );

        let (arr2, arr2_size): ([u8; STUFFED_MESSAGE_SIZE], usize) =
            transceiver.write_structure::<DeviceCommand>(&second_command);

        for k in 0..arr2_size {
            transceiver.add_byte(arr2[k]);
        }

        let received_command_one: Option<DeviceCommand> =
            transceiver.read_structure::<DeviceCommand>();
        let received_command_two: Option<DeviceCommand> =
            transceiver.read_structure::<DeviceCommand>();

        assert!(received_command_one.is_some() && received_command_one.unwrap().eq(&first_command));
        assert!(
            received_command_two.is_some() && received_command_two.unwrap().eq(&second_command)
        );
    }

    #[test]
    fn test_corrupted_data_byte_in_first_message() {
        let mut transceiver = HdlcTransceiver::new();

        let first_command: DeviceCommand = DeviceCommand::new(
            DeviceCommand::CommandType::ChangeMode,
            Some(FSMState::ManualMode),
        );

        let (mut arr, arr_size): ([u8; STUFFED_MESSAGE_SIZE], usize) =
            transceiver.write_structure::<DeviceCommand>(&first_command);

        // corrupt a random byte in the message
        arr[4] ^= 0xFA;

        for k in 0..arr_size {
            transceiver.add_byte(arr[k]);
        }

        let second_command: DeviceCommand = DeviceCommand::new(
            DeviceCommand::CommandType::ChangeMode,
            Some(FSMState::YawControl),
        );

        let (arr2, arr2_size): ([u8; STUFFED_MESSAGE_SIZE], usize) =
            transceiver.write_structure::<DeviceCommand>(&second_command);

        for k in 0..arr2_size {
            transceiver.add_byte(arr2[k]);
        }

        let received_command_one: Option<DeviceCommand> =
            transceiver.read_structure::<DeviceCommand>();
        let received_command_two: Option<DeviceCommand> =
            transceiver.read_structure::<DeviceCommand>();

        // the incorrect message is discarded, and the rest of the messages are correctly received
        assert!(received_command_one.is_none());
        assert!(
            received_command_two.is_some() && received_command_two.unwrap().eq(&second_command)
        );
    }

    #[test]
    fn test_corrupted_message_no_first_frame_boundary() {
        let mut transceiver = HdlcTransceiver::new();

        let first_command: DeviceCommand = DeviceCommand::new(
            DeviceCommand::CommandType::ChangeMode,
            Some(FSMState::RawSensorsFullControlMode),
        );

        let (mut arr, arr_size): ([u8; STUFFED_MESSAGE_SIZE], usize) =
            transceiver.write_structure::<DeviceCommand>(&first_command);

        //remove the frame boundary byte
        arr[0] ^= (1 << 4);

        for k in 0..arr_size {
            transceiver.add_byte(arr[k]);
        }

        let second_command: DeviceCommand = DeviceCommand::new(
            DeviceCommand::CommandType::ChangeMode,
            Some(FSMState::WirelessMode),
        );

        let (arr2, arr2_size): ([u8; STUFFED_MESSAGE_SIZE], usize) =
            transceiver.write_structure::<DeviceCommand>(&second_command);

        for k in 0..arr2_size {
            transceiver.add_byte(arr2[k]);
        }

        let received_command_one: Option<DeviceCommand> =
            transceiver.read_structure::<DeviceCommand>();

        // the first message is completely ignored and we only expect the second message to be
        // received correctly
        //
        // we need to discard just one message to recover from a left frame boundary error
        assert!(
            received_command_one.is_some() && received_command_one.unwrap().eq(&second_command)
        );
    }

    #[test]
    fn test_corrupted_message_no_last_frame_boundary() {
        let mut transceiver = HdlcTransceiver::new();

        let first_command: DeviceCommand = DeviceCommand::new(
            DeviceCommand::CommandType::ChangeMode,
            Some(FSMState::PanicMode),
        );

        let (mut arr, arr_size): ([u8; STUFFED_MESSAGE_SIZE], usize) =
            transceiver.write_structure::<DeviceCommand>(&first_command);

        //remove the frame boundary byte on the right
        arr[arr_size - 1] ^= 1 << 4;

        for k in 0..arr_size {
            transceiver.add_byte(arr[k]);
        }

        let second_command: DeviceCommand = DeviceCommand::new(
            DeviceCommand::CommandType::ChangeMode,
            Some(FSMState::HeightControlMode),
        );

        let (arr2, arr2_size): ([u8; STUFFED_MESSAGE_SIZE], usize) =
            transceiver.write_structure::<DeviceCommand>(&second_command);

        for k in 0..arr2_size {
            transceiver.add_byte(arr2[k]);
        }

        let third_command: DeviceCommand = DeviceCommand::new(
            DeviceCommand::CommandType::ChangeMode,
            Some(FSMState::FullControlMode),
        );

        let (arr3, arr3_size): ([u8; STUFFED_MESSAGE_SIZE], usize) =
            transceiver.write_structure::<DeviceCommand>(&third_command);

        for k in 0..arr3_size {
            transceiver.add_byte(arr3[k]);
        }
        //    boundary is lost
        //         |
        //         v
        // |------(|)|------||-------|

        // one will be received non-matching crc
        let received_command_one: Option<DeviceCommand> =
            transceiver.read_structure::<DeviceCommand>();
        let received_command_three: Option<DeviceCommand> =
            transceiver.read_structure::<DeviceCommand>();

        // the first message is completely ignored, the second messages' bytes are discarded
        // and we expect the third message to be received correctly

        // in other words, when a right frame boundary is lost, we need to discard two messages
        // to recover from the error
        assert!(
            received_command_three.is_some() && received_command_three.unwrap().eq(&third_command)
        );
    }

    #[test]
    fn test_corrupted_message_inserted_boundary() {
        let mut transceiver = HdlcTransceiver::new();

        let first_command: DeviceCommand = DeviceCommand::new(
            DeviceCommand::CommandType::ChangeMode,
            Some(FSMState::ManualMode),
        );

        let (mut arr, arr_size): ([u8; STUFFED_MESSAGE_SIZE], usize) =
            transceiver.write_structure::<DeviceCommand>(&first_command);

        //add a frame boundary byte incorrectly inside a message
        arr[3] = FRAME_BOUNDARY;

        for k in 0..arr_size {
            transceiver.add_byte(arr[k]);
        }

        let second_command: DeviceCommand = DeviceCommand::new(
            DeviceCommand::CommandType::ChangeMode,
            Some(FSMState::HeightControlMode),
        );

        let (arr2, arr2_size): ([u8; STUFFED_MESSAGE_SIZE], usize) =
            transceiver.write_structure::<DeviceCommand>(&second_command);

        for k in 0..arr2_size {
            transceiver.add_byte(arr2[k]);
        }

        // new boundary is incorrectly inserted
        //    |
        //    v
        // |-(|)---||------|

        // one will be received non-matching crc
        let received_command_one: Option<DeviceCommand> =
            transceiver.read_structure::<DeviceCommand>();
        let received_command_two: Option<DeviceCommand> =
            transceiver.read_structure::<DeviceCommand>();

        // a smaller message is first read and discarded since message size is incorrect and
        // furthermore the crc would not match

        // when a frame boundary is incorrectly inserted in the middle of a message, the message
        // after it is correctly received.
        assert!(received_command_one.is_none());
        assert!(
            received_command_two.is_some() && received_command_two.unwrap().eq(&second_command)
        );
    }
}
