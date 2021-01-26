use std::time::Duration;

use rusb::{Context, Device, UsbContext};
use thiserror::Error;
use crate::{Frame, DeviceStatus};

type Result<T> = std::result::Result<T, NativeHeliosError>;

const SDK_VERSION: u8 = 6;

const HELIOS_VID: u16 = 0x1209;
const HELIOS_PID: u16 = 0xE500;

const HELIOS_MAX_POINTS: u32 = 0x1000;
const HELIOS_MAX_RATE: u32 = 0xFFFF;
const HELIOS_MIN_RATE: u32 = 7;

const FRAME_BUFFER_SIZE: u32 = HELIOS_MAX_POINTS * 7 + 5;

// Interrupt endpoints
const ENDPOINT_BULK_OUT: u8 = 0x02;
const ENDPOINT_BULK_IN: u8 = 0x81;
const ENDPOINT_INT_OUT: u8 = 0x06;
const ENDPOINT_INT_IN: u8 = 0x83;

// Interrupt control bytes
const CONTROL_STOP: u8 = 0x01;
const CONTROL_SET_SHUTTER: u8 = 0x02;
const CONTROL_GET_STATUS: u8 = 0x03;
const CONTROL_GET_FIRMWARE_VERSION: u8 = 0x04;
const CONTROL_GET_NAME: u8 = 0x05;
const CONTROL_SET_NAME: u8 = 0x06;
const CONTROL_SEND_SDK_VERSION: u8 = 0x07;

pub struct NativeHeliosDacController {
    context: rusb::Context,
}

impl NativeHeliosDacController {
    pub fn new() -> Result<Self> {
        Ok(NativeHeliosDacController {
            context: rusb::Context::new()?
        })
    }

    pub fn list_devices(&self) -> Result<Vec<NativeHeliosDac>> {
        let devices = self.context.devices()?;
        let mut dacs = vec![];
        for device in devices.iter() {
            let descriptor = device.device_descriptor()?;
            if descriptor.vendor_id() != HELIOS_VID ||
                descriptor.product_id() != HELIOS_PID {
                continue;
            }
            let dac = device.into();
            dacs.push(dac);
        }

        Ok(dacs)
    }
}

pub enum NativeHeliosDac {
    Idle(rusb::Device<rusb::Context>),
    Open {
        device: rusb::Device<rusb::Context>,
        handle: rusb::DeviceHandle<rusb::Context>,
    },
}

impl NativeHeliosDac {
    pub fn open(self) -> Result<Self> {
        match self {
            NativeHeliosDac::Idle(device) => {
                let mut handle = device.open()?;
                handle.claim_interface(0)?;
                handle.set_alternate_setting(0, 1)?;
                let device = NativeHeliosDac::Open {
                    device,
                    handle,
                };

                let firmware = device.firmware_version()?;
                device.send_sdk_version()?;

                Ok(device)
            }
            open => Ok(open)
        }
    }

    /// writes and outputs a frame to the dac
    pub fn write_frame(&mut self, frame: Frame) -> Result<()> {
        if let NativeHeliosDac::Open { handle, .. } = self {
            let mut frame_buffer = Vec::new();

            // this is a bug workaround, the mcu won't correctly receive transfers with these sizes
            let mut pps_actual = frame.pps;
            let mut num_of_points_actual = frame.points.len();
            if (frame.points.len() - 45) % 64 == 0 {
                num_of_points_actual -= 1;
                // adjust pps to keep the same frame duration even with one less point
                pps_actual = frame.pps * ((num_of_points_actual as f32) / (frame.points.len() as f32) + 0.5f32) as u32;
            }

            for point in frame.points {
                frame_buffer.push((point.coordinate.x >> 4) as u8);
                frame_buffer.push(((point.coordinate.x & 0x0F) << 4) as u8 | (point.coordinate.y >> 8) as u8);
                frame_buffer.push((point.coordinate.y & 0xFF) as u8);
                frame_buffer.push(point.color.r);
                frame_buffer.push(point.color.g);
                frame_buffer.push(point.color.b);
                frame_buffer.push(point.intensity);
            }
            frame_buffer.push((pps_actual & 0xFF) as u8);
            frame_buffer.push((pps_actual >> 8) as u8);
            frame_buffer.push((num_of_points_actual & 0xFF) as u8);
            frame_buffer.push((num_of_points_actual >> 8) as u8);
            frame_buffer.push(0); // flags

            let timeout = (8 + frame_buffer.len() >> 5) as u64;
            handle.write_bulk(ENDPOINT_BULK_OUT, &frame_buffer[0..], Duration::from_millis(timeout))?;

            Ok(())
        }else {
            Err(NativeHeliosError::DeviceNotOpened)
        }
    }

    /// Gets name of dac
    pub fn name(&self) -> Result<String> {
        let ctrl_buffer = [CONTROL_GET_NAME, 0];
        let (buffer, _) = self.call_control(&ctrl_buffer)?;

        match buffer {
            [0x85, bytes @ ..] => {
                let null_byte_position = bytes.iter().position(|b| *b == 0u8).unwrap_or(31); // max length is 30 chars
                let (bytes_until_null, _) = bytes.split_at(null_byte_position);
                let name = String::from_utf8(bytes_until_null.to_vec())?;

                Ok(name)
            }
            _ => Err(NativeHeliosError::InvalidDeviceResult)
        }
    }

    /// Get firmware version
    pub fn firmware_version(&self) -> Result<u32> {
        let ctrl_buffer = [CONTROL_GET_FIRMWARE_VERSION, 0];
        let (buffer, size) = self.call_control(&ctrl_buffer)?;

        match &buffer[0..size] {
            [0x84, buffer @ ..] => {
                let buffer = buffer.iter().map(|byte| u32::from(*byte)).collect::<Vec<_>>();
                let version = (buffer[0] << 0) | (buffer[1] << 8) | (buffer[2] << 16) | (buffer[3] << 24);

                Ok(version)
            }
            _ => Err(NativeHeliosError::InvalidDeviceResult)
        }
    }

    fn send_sdk_version(&self) -> Result<()> {
        let ctrl_buffer = [CONTROL_SEND_SDK_VERSION, SDK_VERSION];
        self.send_control(&ctrl_buffer)
    }

    pub fn status(&self) -> Result<DeviceStatus> {
        let ctrl_buffer = [CONTROL_GET_STATUS, 0];
        let (buffer, size) = self.call_control(&ctrl_buffer)?;

        match &buffer[0..size] {
            [0x83, 0] => Ok(DeviceStatus::NotReady),
            [0x83, 1] => Ok(DeviceStatus::Ready),
            _ => Err(NativeHeliosError::InvalidDeviceResult)
        }
    }

    /// Stops output of DAC
    pub fn stop(&self) -> Result<()> {
        let ctrl_buffer = [CONTROL_STOP, 0];
        self.send_control(&ctrl_buffer)
    }

    fn call_control(&self, buffer: &[u8]) -> Result<([u8; 32], usize)> {
        self.send_control(buffer)?;
        self.read_response()
    }

    fn send_control(&self, buffer: &[u8]) -> Result<()> {
        if let NativeHeliosDac::Open { handle, .. } = self {
            let written_length = handle.write_interrupt(ENDPOINT_INT_OUT, buffer, Duration::from_millis(16))?;
            assert_eq!(written_length, buffer.len());

            Ok(())
        }else {
            Err(NativeHeliosError::DeviceNotOpened)
        }
    }

    fn read_response(&self) -> Result<([u8; 32], usize)> {
        if let NativeHeliosDac::Open { handle, .. } = self {
            let mut buffer: [u8; 32] = [0; 32];
            let size = handle.read_interrupt(ENDPOINT_INT_IN, &mut buffer, Duration::from_millis(32))?;

            Ok((buffer, size))
        }else {
            Err(NativeHeliosError::DeviceNotOpened)
        }
    }
}

impl From<rusb::Device<rusb::Context>> for NativeHeliosDac {
    fn from(device: Device<Context>) -> Self {
        NativeHeliosDac::Idle(device)
    }
}

#[derive(Error, Debug)]
pub enum NativeHeliosError {
    #[error("device is not opened")]
    DeviceNotOpened,
    #[error("usb connection error: {0}")]
    UsbError(#[from] rusb::Error),
    #[error("usb device answered with invalid data")]
    InvalidDeviceResult,
    #[error("could not parse string: {0}")]
    Utf8Error(#[from] std::string::FromUtf8Error),
}
