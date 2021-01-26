use crate::wrapper::{HeliosDacWrapper, HeliosError};
use crate::frame::Frame;
use crate::DeviceStatus;

type Result<T> = std::result::Result<T, HeliosError>;

pub struct HeliosDacController {
    wrapper: HeliosDacWrapper
}

impl HeliosDacController {
    pub fn new() -> Self {
        HeliosDacController {
            wrapper: HeliosDacWrapper::new()
        }
    }

    /// Get list of connected dacs.
    pub fn devices(&mut self) -> Result<Vec<HeliosDacDevice>> {
        let device_count = self.wrapper.open_devices()?;
        let mut devices = vec![];
        for i in 0..device_count {
            let name = self.wrapper.name(i)?;
            let device = HeliosDacDevice {
                device_number: i,
                name,
            };

            devices.push(device);
        }

        Ok(devices)
    }
}

pub struct HeliosDacDevice {
    device_number: u32,
    pub name: String,
}

impl std::fmt::Debug for HeliosDacDevice {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        f.debug_struct("HeliosDacDevice")
            .field("name", &self.name)
            .finish()
    }
}

impl HeliosDacDevice {
    /// Writes and outputs a frame
    pub fn write_frame(&self, controller: &mut HeliosDacController, frame: Frame) -> Result<()> {
        controller.wrapper.write_frame(self.device_number, frame)
    }

    /// Gets status of DAC
    pub fn status(&self, controller: &mut HeliosDacController) -> Result<DeviceStatus> {
        controller.wrapper.status(self.device_number)
    }

    /// Returns firmware version of DAC
    pub fn firmware_version(&self, controller: &mut HeliosDacController) -> Result<u32> {
        controller.wrapper.firmware_version(self.device_number)
    }

    /// Sets name of DAC (name must be max 31 characters)
    pub fn set_name(&mut self, controller: &mut HeliosDacController, name: String) -> Result<()> {
        controller.wrapper.set_name(self.device_number, name.clone())?;
        self.name = name;

        Ok(())
    }

    /// Stops output of DAC until new frame is written (NB: blocks for 100ms)
    pub fn stop(&self, controller: &mut HeliosDacController) -> Result<()> {
        controller.wrapper.stop(self.device_number)
    }

    /// Sets shutter level of DAC
    pub fn set_shutter(&self, controller: &mut HeliosDacController, level: bool) -> Result<()> {
        controller.wrapper.set_shutter(self.device_number, level)
    }

    /// Erase the firmware of the DAC, allowing it to be updated by accessing the SAM-BA bootloader
    pub fn erase_firmware(&self, controller: &mut HeliosDacController) -> Result<()> {
        controller.wrapper.erase_firmware(self.device_number)
    }
}
