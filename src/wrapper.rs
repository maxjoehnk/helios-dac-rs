use helios_dac_sys::*;
use crate::frame::*;
use crate::DeviceStatus;

pub struct HeliosDacWrapper {
    dac: HeliosDac,
}

impl HeliosDacWrapper {
    pub fn new() -> Self {
        let dac = unsafe { HeliosDac::new() };

        HeliosDacWrapper { dac }
    }

    /// Initializes drivers, opens connection to all devices
    /// Returns number of all available devices.
    /// NB: To re-scan for newly connected DACs after this function has once been called before, you must first call CloseDevices()
    pub fn open_devices(&mut self) -> Result<u32, HeliosError> {
        let device_count = unsafe { self.dac.OpenDevices() };

        if device_count < 0 {
            parse_error(device_count)?;
        }
        Ok(device_count as u32)
    }

    /// Closes and frees all devices
    pub fn close_devices(&mut self) -> Result<(), HeliosError> {
        let result = unsafe { self.dac.CloseDevices() };

        parse_error(result)
    }

    /// Writes and outputs a frame to the specified dac
    ///
    /// device_number: dac number (0 to n where n+1 is the return value from [open_devices](HeliosDacWrapper::open_devices) )
    pub fn write_frame(&mut self, device_number: u32, frame: Frame) -> Result<(), HeliosError> {
        let mut points = frame.points.into_iter().map(HeliosPoint::from).collect::<Vec<_>>();
        let point_count = points.len() as u32;
        let point_ptr = points.as_mut_ptr();
        let result = unsafe { self.dac.WriteFrame(device_number, frame.pps, 0, point_ptr, point_count) };

        parse_error(result)
    }

    /// Gets status of DAC
    pub fn status(&mut self, device_number: u32) -> Result<DeviceStatus, HeliosError> {
        let status = unsafe { self.dac.GetStatus(device_number) };
        if status < 0 {
            parse_error(status)?;
        }
        match status {
            0 => Ok(DeviceStatus::NotReady),
            1 => Ok(DeviceStatus::Ready),
            _ => unreachable!("invalid status returned")
        }
    }

    /// Returns firmware version of DAC
    pub fn firmware_version(&mut self, device_number: u32) -> Result<u32, HeliosError> {
        let result = unsafe { self.dac.GetFirmwareVersion(device_number) };
        if result < 0 {
            parse_error(result)?;
        }

        Ok(result as u32)
    }

    /// Gets name of DAC (populates name with max 32 characters)
    pub fn name(&mut self, device_number: u32) -> Result<String, HeliosError> {
        let mut name = String::new();
        let name_ptr = name.as_mut_ptr() as *mut i8;
        let result = unsafe { self.dac.GetName(device_number, name_ptr) };
        parse_error(result)?;

        Ok(name)
    }

    /// Sets name of DAC (name must be max 31 characters incl. null terminator)
    pub fn set_name(&mut self, device_number: u32, mut name: String) -> Result<(), HeliosError> {
        // TODO: add max char check
        let name_ptr = name.as_mut_ptr() as *mut i8;
        let result = unsafe { self.dac.SetName(device_number, name_ptr) };

        parse_error(result)
    }

    /// Stops output of DAC until new frame is written (NB: blocks for 100ms)
    pub fn stop(&mut self, device_number: u32) -> Result<(), HeliosError> {
        let result = unsafe { self.dac.Stop(device_number) };

        parse_error(result)
    }

    /// Sets shutter level of DAC
    pub fn set_shutter(&mut self, device_number: u32, level: bool) -> Result<(), HeliosError> {
        let result = unsafe { self.dac.SetShutter(device_number, level) };

        parse_error(result)
    }

    /// Erase the firmware of the DAC, allowing it to be updated by accessing the SAM-BA bootloader
    pub fn erase_firmware(&mut self, device_number: u32) -> Result<(), HeliosError> {
        let result = unsafe { self.dac.EraseFirmware(device_number) };

        parse_error(result)
    }
}

fn parse_error(code: i32) -> Result<(), HeliosError> {
    if code == helios_dac_sys::HELIOS_SUCCESS as i32 {
        Ok(())
    }else {
        unimplemented!("error parsing not done yet")
    }
}

#[derive(Debug)]
pub enum HeliosError {
    /// Attempted to perform an action before calling OpenDevices()
    NotInitialized = HELIOS_ERROR_NOT_INITIALIZED as isize,
    /// Attempted to perform an action with an invalid device number
    InvalidDevNum = HELIOS_ERROR_INVALID_DEVNUM as isize,
    /// [write_frame](HeliosDacWrapper::write_frame) called with null pointer to points
    NullPoints = HELIOS_ERROR_NULL_POINTS as isize,
    /// [write_frame](HeliosDacWrapper::write_frame) called with a frame containing too many points
    TooManyPoints = HELIOS_ERROR_TOO_MANY_POINTS as isize,
    /// [write_frame](HeliosDacWrapper::write_frame) called with pps higher than maximum allowed
    PPSTooHigh = HELIOS_ERROR_PPS_TOO_HIGH as isize,
    /// [write_frame](HeliosDacWrapper::write_frame) called with pps lower than minimum allowed
    PPSTooLow = HELIOS_ERROR_PPS_TOO_LOW as isize,
    /// Attempted to perform an operation on a closed DAC device
    DeviceClosed = HELIOS_ERROR_DEVICE_CLOSED as isize,
    /// Attempted to send a new frame with HELIOS_FLAGS_DONT_BLOCK before previous DoFrame() completed
    DeviceFrameReady = HELIOS_ERROR_DEVICE_FRAME_READY as isize,
    /// Operation failed because SendControl() failed (if operation failed because of libusb_interrupt_transfer failure, the error code will be a libusb error instead)
    DeviceSendControl = HELIOS_ERROR_DEVICE_SEND_CONTROL as isize,
    /// Received an unexpected result from a call to SendControl()
    DeviceResult = HELIOS_ERROR_DEVICE_RESULT as isize,
    /// Attempted to call SendControl() with a null buffer pointer
    DeviceNullBuffer = HELIOS_ERROR_DEVICE_NULL_BUFFER as isize,
    /// Attempted to call SendControl() with a control signal that is too long
    DeviceSignalTooLong = HELIOS_ERROR_DEVICE_SIGNAL_TOO_LONG as isize,
    /// Errors from libusb are the libusb error code added to -5000.
    Libusb = HELIOS_ERROR_LIBUSB_BASE as isize
}

impl Drop for HeliosDacWrapper {
    fn drop(&mut self) {
        unsafe {
            self.dac.destruct();
        }
    }
}

impl From<Point> for HeliosPoint {
    fn from(point: Point) -> Self {
        HeliosPoint {
            x: point.coordinate.x,
            y: point.coordinate.y,
            r: point.color.r,
            g: point.color.g,
            b: point.color.b,
            i: point.intensity
        }
    }
}
