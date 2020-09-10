use core::fmt::Debug;
use device_driver::ll::register::RegisterInterface;
use device_driver::{create_low_level_device, implement_registers, Bit};
use embedded_hal::blocking::{i2c, spi};
use embedded_hal::digital::v2::OutputPin;
use num_enum::{IntoPrimitive, TryFromPrimitive};

#[derive(Debug)]
pub enum HardwareInterfaceError {
    CommunicationError,
    CsError,
}

pub struct I2cInterface<I2C: i2c::WriteRead + i2c::Write> {
    communication_bus: I2C,
    device_address: u8,
}

impl<I2C: i2c::WriteRead + i2c::Write> I2cInterface<I2C> {
    pub fn new(communication_bus: I2C, device_address: u8) -> Self {
        Self {
            communication_bus,
            device_address,
        }
    }

    pub fn free(self) -> I2C {
        self.communication_bus
    }
}

impl<I2C: i2c::WriteRead + i2c::Write> RegisterInterface for I2cInterface<I2C> {
    type Address = u8;
    type InterfaceError = HardwareInterfaceError;

    fn read_register(
        &mut self,
        address: Self::Address,
        value: &mut [u8],
    ) -> Result<(), Self::InterfaceError> {
        self.communication_bus
            .write_read(self.device_address, &[address], value)
            .map_err(|_| HardwareInterfaceError::CommunicationError)
    }

    fn write_register(
        &mut self,
        address: Self::Address,
        value: &[u8],
    ) -> Result<(), Self::InterfaceError> {
        for (i, val) in value.iter().enumerate() {
            self.communication_bus
                .write(self.device_address, &[address - i as u8, *val])
                .map_err(|_| HardwareInterfaceError::CommunicationError)?;
        }
        Ok(())
    }
}

pub struct SpiInterface<SPI: spi::Write<u8> + spi::Transfer<u8>, CS: OutputPin> {
    communication_bus: SPI,
    cs_pin: CS,
    page_status: bool,
}

impl<SPI: spi::Write<u8> + spi::Transfer<u8>, CS: OutputPin> SpiInterface<SPI, CS> {
    pub fn new(communication_bus: SPI, cs_pin: CS) -> Self {
        Self {
            communication_bus,
            cs_pin,
            page_status: false,
        }
    }

    pub fn free(self) -> (SPI, CS) {
        (self.communication_bus, self.cs_pin)
    }
}

impl<SPI: spi::Write<u8> + spi::Transfer<u8>, CS: OutputPin> RegisterInterface
    for SpiInterface<SPI, CS>
{
    type Address = u8;
    type InterfaceError = HardwareInterfaceError;

    fn read_register(
        &mut self,
        address: Self::Address,
        value: &mut [u8],
    ) -> Result<(), Self::InterfaceError> {
        self.cs_pin
            .set_low()
            .map_err(|_| HardwareInterfaceError::CsError)?;
        self.communication_bus
            .write(&[address | 0x80])
            .map_err(|_| HardwareInterfaceError::CommunicationError)?;
        self.communication_bus
            .transfer(value)
            .map_err(|_| HardwareInterfaceError::CommunicationError)?;
        self.cs_pin
            .set_high()
            .map_err(|_| HardwareInterfaceError::CsError)?;
        Ok(())
    }

    fn write_register(
        &mut self,
        address: Self::Address,
        value: &[u8],
    ) -> Result<(), Self::InterfaceError> {
        if address != 0x73 {
            // We may need to switch page
            if address < 0x80 && !self.page_status {
                // Set the page high
                self.write_register(0x73, &[0x10])?;
                self.page_status = true;
            }
            if address >= 0x80 && self.page_status {
                // Set the page high
                self.write_register(0x73, &[0x00])?;
                self.page_status = false;
            }
        }

        self.cs_pin
            .set_low()
            .map_err(|_| HardwareInterfaceError::CsError)?;
        for (i, val) in value.iter().enumerate() {
            self.communication_bus
                .write(&[(address - i as u8) & 0x7F, *val])
                .map_err(|_| HardwareInterfaceError::CommunicationError)?;
        }
        self.cs_pin
            .set_high()
            .map_err(|_| HardwareInterfaceError::CsError)?;
        Ok(())
    }
}

// Create our low level device. This holds all the hardware communication definitions
create_low_level_device!({
    // The name of our low level device
    name: Bme680,
    // The types of errors our low level error enum must contain
    errors: [HardwareInterfaceError],
});

// Create a register set for the device
implement_registers!(
    /// The global register set
    Bme680.registers<u8> = {
        status(RW, 0x73, 1) = {
            /// In SPI mode complete memory page is accessed using page 0 & page 1. Register spi_mem_page is used for page selection.
            /// After power-on, spi_mem_page is in its reset state and page 0(0x80 to 0xFF) will be active. Page1 (0x00 to 0x7F) will be
            /// active on setting spi_mem_page. Please refer Table 19 for better understanding.
            spi_mem_page: u8 as Bit = RW 4..=4,
        },
        reset(RW, 0xE0, 1) = {
            /// Writing 0xB6 to this register initiates a soft-reset procedure, which has the same effect like power-on reset. The default
            /// value stored in this register is 0x00.
            value: u8 as Reset = RW 0..8,
        },
        id(RO, 0xD0, 1) = {
            chip_id: u8 = RO 0..8,
        },
        config(RW, 0x75, 1) = {
            /// IIR filter settings
            filter: u8 as FilterCoefficient = RW 2..=4,
            /// Enable SPI 3 wire mode
            spi_3w_en: u8 as Bit = RW 0..=0,
        },
        ctrl_meas(RW, 0x74, 1) = {
            /// Temperature oversampling settings
            osrs_t: u8 as Oversampling = RW 5..=7,
            /// Pressure oversampling settings
            osrs_p: u8 as Oversampling = RW 2..=4,
            /// Select sensor power mode as shown in the following table
            mode: u8 as Mode = RW 0..=1,
        },
        ctrl_hum(RW, 0x72, 1) = {
            /// New data interrupt can be enabled if the device is in
            /// SPI 3 wire mode and pi_3w_int_en=1.
            /// The new data interrupt is then indicated on the SDO pad.
            spi_3w_int_en: u8 as Bit = RW 6..=6,
            /// Controls over sampling setting of humidity sensor
            osrs_h: u8 as Oversampling = RW 0..=2,
        },
        ctrl_gas_1(RW, 0x71, 1) = {
            /// The gas conversions are started only in appropriate mode if run_gas = ‘1’
            run_gas: u8 as Bit = RW 4..=4,
            /// Indicates index of heater set point that will be used in forced mode
            nb_conv: u8 as HeaterProfile = RW 0..=3,
        },
        ctrl_gas_0(RW, 0x70, 1) = {
            /// Turn off current injected to heater by setting bit to one
            heat_off: u8 as Bit = RW 3..=3,
        },

        gas_wait_9(RW, 0x6D, 1) = {
            multiplication: u8 as GasSensorWaitMultiplicationFactor = RW 6..=7,
            value: u8 = RW 0..=5,
        },
        gas_wait_8(RW, 0x6C, 1) = {
            multiplication: u8 as GasSensorWaitMultiplicationFactor = RW 6..=7,
            value: u8 = RW 0..=5,
        },
        gas_wait_7(RW, 0x6B, 1) = {
            multiplication: u8 as GasSensorWaitMultiplicationFactor = RW 6..=7,
            value: u8 = RW 0..=5,
        },
        gas_wait_6(RW, 0x6A, 1) = {
            multiplication: u8 as GasSensorWaitMultiplicationFactor = RW 6..=7,
            value: u8 = RW 0..=5,
        },
        gas_wait_5(RW, 0x69, 1) = {
            multiplication: u8 as GasSensorWaitMultiplicationFactor = RW 6..=7,
            value: u8 = RW 0..=5,
        },
        gas_wait_4(RW, 0x68, 1) = {
            multiplication: u8 as GasSensorWaitMultiplicationFactor = RW 6..=7,
            value: u8 = RW 0..=5,
        },
        gas_wait_3(RW, 0x67, 1) = {
            multiplication: u8 as GasSensorWaitMultiplicationFactor = RW 6..=7,
            value: u8 = RW 0..=5,
        },
        gas_wait_2(RW, 0x66, 1) = {
            multiplication: u8 as GasSensorWaitMultiplicationFactor = RW 6..=7,
            value: u8 = RW 0..=5,
        },
        gas_wait_1(RW, 0x65, 1) = {
            multiplication: u8 as GasSensorWaitMultiplicationFactor = RW 6..=7,
            value: u8 = RW 0..=5,
        },
        gas_wait_0(RW, 0x64, 1) = {
            multiplication: u8 as GasSensorWaitMultiplicationFactor = RW 6..=7,
            value: u8 = RW 0..=5,
        },

        res_heat_9(RW, 0x63, 1) = {
            value: u8 = RW 0..8,
        },
        res_heat_8(RW, 0x62, 1) = {
            value: u8 = RW 0..8,
        },
        res_heat_7(RW, 0x61, 1) = {
            value: u8 = RW 0..8,
        },
        res_heat_6(RW, 0x60, 1) = {
            value: u8 = RW 0..8,
        },
        res_heat_5(RW, 0x5F, 1) = {
            value: u8 = RW 0..8,
        },
        res_heat_4(RW, 0x5E, 1) = {
            value: u8 = RW 0..8,
        },
        res_heat_3(RW, 0x5D, 1) = {
            value: u8 = RW 0..8,
        },
        res_heat_2(RW, 0x5C, 1) = {
            value: u8 = RW 0..8,
        },
        res_heat_1(RW, 0x5B, 1) = {
            value: u8 = RW 0..8,
        },
        res_heat_0(RW, 0x5A, 1) = {
            value: u8 = RW 0..8,
        },

        idac_heat_9(RW, 0x59, 1) = {
            value: u8 = RW 0..8,
        },
        idac_heat_8(RW, 0x58, 1) = {
            value: u8 = RW 0..8,
        },
        idac_heat_7(RW, 0x57, 1) = {
            value: u8 = RW 0..8,
        },
        idac_heat_6(RW, 0x56, 1) = {
            value: u8 = RW 0..8,
        },
        idac_heat_5(RW, 0x55, 1) = {
            value: u8 = RW 0..8,
        },
        idac_heat_4(RW, 0x54, 1) = {
            value: u8 = RW 0..8,
        },
        idac_heat_3(RW, 0x53, 1) = {
            value: u8 = RW 0..8,
        },
        idac_heat_2(RW, 0x52, 1) = {
            value: u8 = RW 0..8,
        },
        idac_heat_1(RW, 0x51, 1) = {
            value: u8 = RW 0..8,
        },
        idac_heat_0(RW, 0x50, 1) = {
            value: u8 = RW 0..8,
        },

        gas_r(RO, 0x2B, 2) = {
            gas_range: u8 = RO 0..=3,
            /// Heater temperature stability for target heater resistance is indicated heat_stab_x status bits.
            heat_stab_r: u8 as Bit = RO 4..=4,
            /// In each TPHG sequence contains a gas measurement slot, either a real one which result is used or a dummy one to keep
            /// a constant sampling rate and predictable device timing. A real gas conversion (i.e., not a dummy one) is indicated by the
            /// gas_valid_r status register.
            gas_valid_r: u8 as Bit = RO 5..=5,
            value: u16 = RO 6..16,
        },
        hum_r(RO, 0x26, 2) = {
            value: u16 = RO 0..16,
        },
        temp(RO, 0x24, 3) = {
            value: u32 = RO 4..24,
        },
        press(RO, 0x21, 3) = {
            value: u32 = RO 4..24,
        },
        eas_status_0(RO, 0x1D, 1) = {
            /// User can program a sequence of up to 10 conversions by setting nb_conv<3:0>. Each conversion has its own heater
            /// resistance target but 3 field registers to store conversion results. The actual gas conversion number in the measurement
            /// sequence (up to 10 conversions numbered from 0 to 9) is stored in gas_meas_index register.
            gas_meas_index_0: u8 = RO 0..=3,
            /// Measuring status will be set to ‘1’ whenever a conversion (temperature, pressure, humidity and gas) is running and back to
            /// ‘0’ when the results have been transferred to the data registers.
            measuring: u8 as Bit = RO 5..=5,
            /// Measuring bit is set to “1‟ only during gas measurements, goes to “0‟ as soon as measurement is completed and data
            /// transferred to data registers. The registers storing the configuration values for the measurement (gas_wait_shared,
            /// gas_wait_x, res_heat_x, idac_heat_x, image registers) should not be changed when the device is measuring.
            gas_measuring: u8 as Bit = RO 6..=6,
            /// The measured data are stored into the output data registers at the end of each TPHG conversion phase along with status
            /// flags and index of measurement.
            new_data: u8 as Bit = RO 7..=7,
        },
    }
);

/// The mode of the chip. 2 bits.
#[repr(u8)]
#[derive(Debug, Copy, Clone, Eq, PartialEq, IntoPrimitive, TryFromPrimitive)]
pub enum Mode {
    Sleep = 0b00,
    Forced = 0b01,
}

/// Reset value. 8 bits.
#[repr(u8)]
#[derive(Debug, Copy, Clone, Eq, PartialEq, IntoPrimitive, TryFromPrimitive)]
pub enum Reset {
    None = 0x00,
    SoftReset = 0xB6,
}

/// Oversampling value. 3 bits.
#[repr(u8)]
#[derive(Debug, Copy, Clone, Eq, PartialEq, IntoPrimitive, TryFromPrimitive)]
pub enum Oversampling {
    Skipped = 0b000,
    OversamplingX1 = 0b001,
    OversamplingX2 = 0b010,
    OversamplingX4 = 0b011,
    OversamplingX8 = 0b100,
    OversamplingX16 = 0b101,
    OversamplingX16Alt1 = 0b110,
    OversamplingX16Alt2 = 0b111,
}

/// Filter coefficient value. 3 bits.
#[repr(u8)]
#[derive(Debug, Copy, Clone, Eq, PartialEq, IntoPrimitive, TryFromPrimitive)]
pub enum FilterCoefficient {
    FC0 = 0b000,
    FC1 = 0b001,
    FC3 = 0b010,
    FC7 = 0b011,
    FC15 = 0b100,
    FC31 = 0b101,
    FC63 = 0b110,
    FC127 = 0b111,
}

/// The gas sensor wait time multiplication factor. 2 bits.
#[repr(u8)]
#[derive(Debug, Copy, Clone, Eq, PartialEq, IntoPrimitive, TryFromPrimitive)]
pub enum GasSensorWaitMultiplicationFactor {
    Factor1 = 0b00,
    Factor4 = 0b01,
    Factor16 = 0b10,
    Factor64 = 0b11,
}

/// Heater profile set-point value. 4 bits.
#[repr(u8)]
#[derive(Debug, Copy, Clone, Eq, PartialEq, IntoPrimitive, TryFromPrimitive)]
pub enum HeaterProfile {
    Setpoint0 = 0,
    Setpoint1 = 1,
    Setpoint2 = 2,
    Setpoint3 = 3,
    Setpoint4 = 4,
    Setpoint5 = 5,
    Setpoint6 = 6,
    Setpoint7 = 7,
    Setpoint8 = 8,
    Setpoint9 = 9,
}
