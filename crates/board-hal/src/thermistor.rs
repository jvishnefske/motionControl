//! Thermistor reading and temperature conversion.
//!
//! Supports NTC thermistors with configurable beta parameter
//! in a voltage divider circuit with known series resistance.

/// Identifies an ADC temperature channel.
#[derive(Clone, Copy, Debug, PartialEq, Eq, defmt::Format)]
pub enum TempChannel {
    /// TEMP_0: Bed thermistor
    Bed,
    /// TEMP_1: Hotend 1 thermistor
    Hotend1,
    /// TEMP_2: Hotend 2 / aux thermistor
    Hotend2,
}

/// Thermistor calibration parameters.
#[derive(Clone, Copy, Debug)]
pub struct ThermistorParams {
    /// Nominal resistance at 25C in ohms (typically 100_000 for 100K NTC).
    pub r_nominal: f32,
    /// Beta coefficient (typically 3950-4300 for common thermistors).
    pub beta: f32,
    /// Series resistor value in ohms (4700 on Duet 3 Mini 5+).
    pub r_series: f32,
    /// ADC resolution (4095 for 12-bit).
    pub adc_max: f32,
}

impl Default for ThermistorParams {
    fn default() -> Self {
        Self {
            r_nominal: 100_000.0,
            beta: 4267.0,
            r_series: 4700.0,
            adc_max: 4095.0,
        }
    }
}

impl ThermistorParams {
    /// Convert a raw 12-bit ADC reading to temperature in degrees Celsius.
    /// Uses the simplified B-parameter Steinhart-Hart equation.
    pub fn adc_to_celsius(&self, adc_raw: u16) -> f32 {
        if adc_raw == 0 || adc_raw >= self.adc_max as u16 {
            return f32::NAN; // open or shorted thermistor
        }

        let r_therm = self.r_series * (adc_raw as f32) / (self.adc_max - adc_raw as f32);

        // Simplified Steinhart-Hart: 1/T = 1/T0 + (1/B) * ln(R/R0)
        let t0_kelvin: f32 = 298.15; // 25C in Kelvin
        let ln_ratio = libm::logf(r_therm / self.r_nominal);
        let inv_t = (1.0 / t0_kelvin) + (1.0 / self.beta) * ln_ratio;
        let temp_kelvin = 1.0 / inv_t;

        temp_kelvin - 273.15
    }
}
