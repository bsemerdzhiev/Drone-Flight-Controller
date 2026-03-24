use nrf51_pac::{Peripherals, RADIO};
use my_hdlc::command::{self, DeviceCommand, DroneInfo, FSMState};
use my_hdlc::pc_command::ManualInput;
use my_hdlc::{HdlcTransceiver, STUFFED_MESSAGE_SIZE};

static mut RX_BUF: [u8; 130] = [0; 130];
static mut TX_BUF: [u8; 130] = [0; 130];

pub fn radio_init() -> RADIO {
    let p = Peripherals::take().unwrap();
    let radio = p.RADIO;

    // set to 1Mbps mode
    radio.mode.write(|w| w.mode().nrf_1mbit());

    // set the RF channel to 7
    radio.frequency.write(|w| unsafe {
        w.frequency().bits(7)
    });

    // 0 output power (no attenuation)
    radio.txpower.write(|w| w.txpower()._0d_bm());

    // logical address set to 0
    radio.txaddress.write(|w| unsafe {
        w.txaddress().bits(0)
    });
    radio.rxaddresses.write(|w| w.addr0().enabled());

    // prefix bytes
    radio.prefix0.write(|w| unsafe {
        w.ap0().bits(0xC3)
    });
    
    // base address
    radio.base0.write(|w| unsafe {
        w.bits(0xE7E7E7E7)
    });

    radio.pcnf0.write(|w| unsafe {
        w.lflen().bits(8).s0len().bit(false).s1len().bits(0)
    });

    radio.pcnf1.write(|w| unsafe {
        w.maxlen().bits(130)
         .statlen().bits(0)
         .balen().bits(3)
         .endian().little()
         .whiteen().enabled()
    });

    // radio.crccnf.write(|w| w.len().two()); // 2 byte crc
    // radio.crcpoly.write(|w| unsafe {
    //     w.crcpoly().bits(0x11021)
    // });
    // radio.crcinit.write(|w| unsafe {
    //     w.crcinit().bits(0xFFFF)
    // });

    radio.shorts.write(|w| w.ready_start().enabled());

    radio
}

pub fn radio_start_rx(radio: &RADIO) {
    // let p = unsafe { Peripherals::steal() };
    // let radio = &p.RADIO;

    unsafe {
        radio.packetptr.write(|w| w.bits(RX_BUF.as_ptr() as u32));
    }

    radio.events_ready.reset();
    radio.events_end.reset();

    radio.tasks_rxen.write(|w| unsafe {
        w.bits(1)
    });
}

pub fn radio_poll_rx(radio: &RADIO, transceiver: &mut HdlcTransceiver) -> Option<DeviceCommand> {
    // let p = unsafe{ Peripherals::steal() };
    // let radio = &p.RADIO;

    if radio.events_end.read().bits() == 0 {
        return None
    }

    radio.events_end.reset();

    let len = unsafe { RX_BUF[0] as usize };
    let payload = unsafe { &RX_BUF[1..1 + len] };

    transceiver.add_bytes(payload);
    transceiver.read_structure::<DeviceCommand>()
}

pub fn radio_send_command(radio: &RADIO, transceiver: &mut HdlcTransceiver, cmd: &DeviceCommand) {
    let (frame, len) = transceiver.write_structure(cmd);

    if len == 0 {
        //frame too large
        return;
    }

    radio_send(radio, &frame[..len]);
}

pub fn radio_send(radio: &RADIO, payload: &[u8]) {
    // let p = unsafe { Peripherals::steal() };
    // let radio = &p.RADIO;

    unsafe {
        TX_BUF[..payload.len()].copy_from_slice(payload);
        radio.packetptr.write(|w| w.bits(TX_BUF.as_ptr() as u32));
    }

    radio.events_ready.reset();
    radio.events_end.reset();

    radio.tasks_txen.write(|w| unsafe { w.bits(1) });

    while radio.events_end.read().bits() == 0 {}
}