//! Pin definitions for the Duet 3 Mini 5+ board.
//!
//! All pin assignments sourced from the Klipper generic-duet3-mini.cfg
//! and the Duet3D hardware documentation.

/// GPIO port/pin identifier for the SAME54.
#[derive(Clone, Copy, Debug, defmt::Format)]
pub struct Pin {
    pub port: Port,
    pub pin: u8,
}

#[derive(Clone, Copy, Debug, defmt::Format)]
pub enum Port {
    A,
    B,
    C,
    D,
}

impl Pin {
    pub const fn new(port: Port, pin: u8) -> Self {
        Self { port, pin }
    }
}

/// Complete pin mapping for Duet 3 Mini 5+.
pub struct Duet3Pins;

impl Duet3Pins {
    // ── Stepper drivers ──────────────────────────────────────────
    // All 5 drivers share enable pin PC28 (active low).
    pub const STEPPER_ENABLE: Pin = Pin::new(Port::C, 28);

    // Driver 0
    pub const STEP_0: Pin = Pin::new(Port::C, 26);
    pub const DIR_0: Pin = Pin::new(Port::B, 3); // inverted in hardware

    // Driver 1
    pub const STEP_1: Pin = Pin::new(Port::C, 25);
    pub const DIR_1: Pin = Pin::new(Port::B, 29);

    // Driver 2
    pub const STEP_2: Pin = Pin::new(Port::C, 24);
    pub const DIR_2: Pin = Pin::new(Port::B, 28);

    // Driver 3
    pub const STEP_3: Pin = Pin::new(Port::C, 19);
    pub const DIR_3: Pin = Pin::new(Port::D, 20);

    // Driver 4
    pub const STEP_4: Pin = Pin::new(Port::C, 16);
    pub const DIR_4: Pin = Pin::new(Port::D, 21);

    // TMC2209 shared UART
    pub const TMC_UART_TX: Pin = Pin::new(Port::A, 1);
    pub const TMC_UART_RX: Pin = Pin::new(Port::A, 0);
    pub const TMC_UART_SEL: Pin = Pin::new(Port::D, 0);

    // ── Heater / PWM outputs ─────────────────────────────────────
    pub const OUT_0_BED: Pin = Pin::new(Port::B, 17);    // Heated bed (15A fused)
    pub const OUT_1_HEATER1: Pin = Pin::new(Port::C, 10); // Hotend 1
    pub const OUT_2_HEATER2: Pin = Pin::new(Port::B, 13); // Hotend 2
    pub const OUT_3_FAN0: Pin = Pin::new(Port::B, 11);    // Heatbreak fan
    pub const OUT_4_FAN1: Pin = Pin::new(Port::A, 11);    // Part cooling fan
    pub const OUT_5_FAN2: Pin = Pin::new(Port::B, 2);     // Aux fan
    pub const OUT_6_FAN3: Pin = Pin::new(Port::B, 1);     // Aux / laser / VFD

    // ── Thermistor ADC inputs ────────────────────────────────────
    pub const TEMP_0: Pin = Pin::new(Port::C, 0);  // Bed thermistor
    pub const TEMP_1: Pin = Pin::new(Port::C, 1);  // Hotend thermistor
    pub const TEMP_2: Pin = Pin::new(Port::C, 2);  // Aux thermistor
    pub const VSSA: Pin = Pin::new(Port::B, 4);     // ADC ground reference
    pub const VREF: Pin = Pin::new(Port::B, 5);     // ADC voltage reference

    // ── Endstop / IO inputs ──────────────────────────────────────
    pub const IO5_IN: Pin = Pin::new(Port::C, 31);  // X endstop (3.3V, pull-up)
    pub const IO6_IN: Pin = Pin::new(Port::C, 4);   // Y endstop (3.3V, pull-up)
    pub const IO1_OUT: Pin = Pin::new(Port::B, 31);
    pub const IO2_OUT: Pin = Pin::new(Port::D, 9);   // Z endstop area

    // ── SPI bus (SERCOM7) ────────────────────────────────────────
    pub const SPI_MOSI: Pin = Pin::new(Port::C, 12);
    pub const SPI_MISO: Pin = Pin::new(Port::C, 15);
    pub const SPI_SCK: Pin = Pin::new(Port::C, 13);

    // ── USB ──────────────────────────────────────────────────────
    pub const USB_TX: Pin = Pin::new(Port::B, 25);
    pub const USB_RX: Pin = Pin::new(Port::B, 24);

    // ── CAN-FD ───────────────────────────────────────────────────
    pub const CAN_TX: Pin = Pin::new(Port::B, 14);
    pub const CAN_RX: Pin = Pin::new(Port::B, 15);

    // ── Status LEDs ──────────────────────────────────────────────
    pub const LED_DIAG: Pin = Pin::new(Port::A, 31);     // Active low
    pub const LED_ACTIVITY: Pin = Pin::new(Port::A, 30);
}
