// ---------------------
// UNUSED
// ---------------------
// Earlier version of wireless mode attempted using radio. Switched to using bluetooth.
// Setup file kept for documentation purposes. 



// use nrf51_pac::{NVIC, Peripherals, RADIO};
// use my_hdlc::command::{self, DeviceCommand, DroneInfo, FSMState};
// use my_hdlc::pc_command::ManualInput;
// use my_hdlc::{HdlcTransceiver, STUFFED_MESSAGE_SIZE};

// make buffer as large as needed. 
// static mut RX_BUF: [u8; 0] = [];
// static mut TX_BUF: [u8; 0] = [];
// const PACKET_SIZE: u8 = 16;

// pub fn radio_init() -> RADIO {
//     // have to steal because peripherals is already taken in the init
//     let p = unsafe {
//         Peripherals::steal()
//     };
//     let clock = p.CLOCK;
//     let radio = p.RADIO;

//     // enable high freqency clock
//     clock.events_hfclkstarted.write(|w| unsafe {
//         w.bits(0)
//     });
//     clock.tasks_hfclkstart.write(|w| unsafe {
//         w.bits(1)
//     });
//     while clock.events_hfclkstarted.read().bits() == 0 {}

//     // transmit power 
//     radio.txpower.write(|w| w.txpower().pos4d_bm());

//     // set the frequency to 2407MHz
//     radio.frequency.write(|w| unsafe {
//         w.frequency().bits(7)
//     });

//     // set to 1Mbps mode
//     radio.mode.write(|w| w.mode().nrf_1mbit());

//     // base address
//     radio.base0.write(|w| unsafe {
//         w.bits(0x75626974)
//     });

//     // prefix bytes
//     radio.prefix0.write(|w| unsafe {
//         w.ap0().bits(0xC3)
//     });

//     // logical address set to 0
//     radio.txaddress.write(|w| unsafe {
//         w.txaddress().bits(0)
//     });
//     radio.rxaddresses.write(|w| w.addr0().enabled());

  
//     radio.pcnf0.write(|w| unsafe {
//         w.lflen().bits(8).s0len().bit(false).s1len().bits(0)
//     });

//     radio.pcnf1.write(|w| unsafe {
//         w.maxlen().bits(PACKET_SIZE)
//          .statlen().bits(0)
//          .balen().bits(3)
//          .endian().little()
//          .whiteen().enabled()
//     });

//     radio.datawhiteiv.write(|w| unsafe {
//         w.bits(0x18)
//     });

//     radio.crccnf.write(|w| w.len().two()); // 2 byte crc
//     radio.crcinit.write(|w| unsafe {
//         w.crcinit().bits(0xFFFF)
//     });
//     radio.crcpoly.write(|w| unsafe {
//         w.crcpoly().bits(0x11021)
//     });

//     radio.shorts.write(|w| w.ready_start().enabled().end_disable().enabled());

//     // unmask interrupts (possible use later)
//     // unsafe {
//     //     NVIC::unmask(nrf51_pac::Interrupt::RADIO);
//     // }

//     radio
// }

// pub fn radio_start_rx(radio_option: Option<&RADIO>) {
//     // let p = unsafe { Peripherals::steal() };
//     // let radio = &p.RADIO;
//     if let Some(radio) = radio_option {
//         unsafe {
//             radio.packetptr.write(|w| w.bits(RX_BUF.as_ptr() as u32));
//         }

//         radio.events_ready.reset();
//         radio.events_end.reset();
//         radio.events_address.reset();

//         radio.tasks_rxen.write(|w| unsafe {
//             w.bits(1)
//         });

//         // wait for ready
//         while radio.events_ready.read().bits() == 0 {}
//     }
// }

// // poll the receiving end
// pub fn radio_poll_rx(radio_option: Option<&RADIO>, transceiver: &mut HdlcTransceiver) -> Option<DeviceCommand> {
//     // let p = unsafe{ Peripherals::steal() };
//     // let radio = &p.RADIO;
//     if let Some(radio) = radio_option {
//         if radio.events_end.read().bits() == 0 {
//             return None
//         }

//         radio.events_end.reset();

//         let len = unsafe { RX_BUF[0] as usize };
//         let payload = unsafe { &RX_BUF[1..1 + len] };

//         transceiver.add_bytes(payload);
//         transceiver.read_structure::<DeviceCommand>()
//     } else {
//         None
//     }
// }

// // pub fn radio_send_command(radio: &RADIO, transceiver: &mut HdlcTransceiver, cmd: &DeviceCommand) {
// //     let (frame, len) = transceiver.write_structure(cmd);

// //     if len == 0 {
// //         //frame too large
// //         return;
// //     }

// //     radio_send(radio, &frame[..len]);
// // }

// // Function is blocking (no interrupts implemented currently)
// pub fn radio_send(radio_option: Option<&RADIO>, payload: &[u8]) {
//     // let p = unsafe { Peripherals::steal() };
//     // let radio = &p.RADIO;
//     if let Some(radio) = radio_option {
//         unsafe {
//             TX_BUF[..payload.len()].copy_from_slice(payload);
//             radio.packetptr.write(|w| w.bits(TX_BUF.as_ptr() as u32));
//         }

//         radio.events_ready.reset();
//         radio.events_end.reset();
//         radio.events_address.reset();

//         radio.tasks_txen.write(|w| unsafe { w.bits(1) });

//         while radio.events_end.read().bits() == 0 {}
//     }
// }