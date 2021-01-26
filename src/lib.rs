#[cfg(feature = "sdk")]
pub mod wrapper;
#[cfg(feature = "sdk")]
mod device;
#[cfg(feature = "native")]
mod native;
mod frame;

#[cfg(feature = "sdk")]
pub use crate::{device::*, wrapper::{HeliosError, DeviceStatus}};
#[cfg(feature = "native")]
pub use crate::native::*;
pub use crate::frame::*;

#[derive(Debug, Clone, Copy)]
pub enum DeviceStatus {
    /// Device is ready to receive frame
    Ready = 1,
    /// Device is not ready to receive frame
    NotReady = 0
}
