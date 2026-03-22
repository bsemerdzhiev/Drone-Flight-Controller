use nrf51_pac::Peripherals;

static mut RX_BUF: [u8; 32] = [0; 32];
static mut TX_BUF: [u8; 32] = [0; 32];

pub fn radio_init() {
    let p = Peripherals::take().unwrap();
    let radio = &p.RADIO;

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
        w.lflen().bits(0).s0len().bit(true).s1len().bits(0)
    });

    radio.pcnf1.write(|w| unsafe {
        w.maxlen().bits(16)
         .statlen().bits(16)
         .balen().bits(3)
         .endian().little()
         .whiteen().enabled()
    });

    radio.crccnf.write(|w| w.len().two()); // 2 byte crc
    radio.crcpoly.write(|w| unsafe {
        w.crcpoly().bits(0x11021)
    });
    radio.crcinit.write(|w| unsafe {
        w.crcinit().bits(0xFFFF)
    });

    radio.shorts.write(|w| w.ready_start().enabled());
}

pub fn radio_start_rx() {
    let p = unsafe { Peripherals::steal() };
    let radio = &p.RADIO;

    unsafe {
        radio.packetptr.write(|w| w.bits(RX_BUF.as_ptr() as u32));
    }

    radio.events_ready.reset();
    radio.events_end.reset();

    radio.tasks_rxen.write(|w| unsafe {
        w.bits(1)
    });
}

pub fn radio_poll_rx() -> Option<&'static [u8]> {
    let p = unsafe{ Peripherals::steal() };
    let radio = &p.RADIO;

    if radio.events_end.read().bits() != 0 {
        radio.events_end.reset();
        unsafe { return Some(&RX_BUF[..16]); }
    }

    None
}

pub fn radio_send(payload: &[u8]) {
    let p = unsafe { Peripherals::steal() };
    let radio = &p.RADIO;

    unsafe {
        TX_BUF[..payload.len()].copy_from_slice(payload);
        radio.packetptr.write(|w| w.bits(TX_BUF.as_ptr() as u32));
    }

    radio.events_ready.reset();
    radio.events_end.reset();

    radio.tasks_txen.write(|w| unsafe { w.bits(1) });

    while radio.events_end.read().bits() == 0 {}
}