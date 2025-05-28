use core::{
    cell::{Cell, RefCell},
    ops::{DerefMut, Shr}, u16,
};

use cortex_m as cm;

use cm::{
    interrupt::Mutex,
};

use stm32f4xx_hal::{
    prelude::*,
    pac,
    rcc,
    timer,
    interrupt,
};

static ELAPSED_MS: Mutex <Cell<u32>> = Mutex::new(Cell::new(0u32));
static TIMER_TIM3: Mutex <RefCell <Option <timer::CounterMs <pac::TIM3>>>> = Mutex::new(RefCell::new(None));

pub fn init_timer(tim3: pac::TIM3, clocks: &rcc::Clocks) {
    // Millis timer
    let mut timer = tim3.counter(&clocks);
    timer.start(10.millis()).unwrap();
    timer.listen(timer::Event::Update);

    cm::interrupt::free(|v| {
        TIMER_TIM3.borrow(v).replace(Some(timer));
    });

    pac::NVIC::unpend(interrupt::TIM3);

    unsafe {
        pac::NVIC::unmask(pac::Interrupt::TIM3);
    }
}

pub fn get_millis() -> u32 {
    cm::interrupt::free(|v| {
        ELAPSED_MS.borrow(v).get().wrapping_mul(10)
    })
}

pub fn beat_u8(bpm: u16) -> u8 {
    beat_u16(bpm) as u8
}

pub fn beat_u16(bpm: u16) -> u16 {
    // = bpm * (1m / 60s) * t * (1s / 1000ms)
    // = bom * t * (2^16/60000) / 2^16
    // = bpm * t * 280 / (2^8 * 2^16)
    // = bpm * t * 280 / (2^24)
    // To prevent division, we convert 1/60000 into 60000/2^16 * 2^16/60000
    // The ratio (2^16/60000) can be expressed as (280/2^8)
    
    get_millis()
        .saturating_mul(u32::from(bpm))
        .wrapping_shr(16)
        .saturating_mul(280)
        .wrapping_shr(8)
        as u16
}

#[interrupt]
fn TIM3() {
    cm::interrupt::free(|v| {
        if let Some(tim2) = TIMER_TIM3.borrow(v).borrow_mut().deref_mut() {
            tim2.clear_flags(timer::Flag::Update);
        }

        let r = ELAPSED_MS.borrow(v);
        let val = r.get();
        r.replace(val.wrapping_add(1));
    });
}