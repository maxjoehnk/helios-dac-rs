use bitflags::bitflags;

#[derive(Debug, Clone, PartialEq)]
pub struct Frame {
    /// Rate of output in points per second
    pub pps: u32,
    /// default is 0
    pub flags: WriteFrameFlags,
    pub points: Vec<Point>,
}

impl Frame {
    pub fn new(pps: u32, points: Vec<Point>) -> Self {
        Frame {
            pps,
            points,
            flags: WriteFrameFlags::empty(),
        }
    }

    pub fn new_with_flags(pps: u32, points: Vec<Point>, flags: WriteFrameFlags) -> Self {
        Frame {
            pps,
            points,
            flags
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Point {
    pub coordinate: Coordinate,
    pub color: Color,
    pub intensity: u8,
}

/// Coordinates (x, y)
///
/// 12 bit (from 0 to 0xFFF)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Coordinate {
    pub x: u16,
    pub y: u16
}

impl From<(u16, u16)> for Coordinate {
    fn from((x, y): (u16, u16)) -> Self {
        Coordinate { x, y }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Color {
    /// Red channel
    pub r: u8,
    /// Green channel
    pub g: u8,
    /// Blue channel
    pub b: u8,
}

impl Color {
    pub fn new(r: u8, g: u8, b: u8) -> Self {
        Color {
            r,
            g,
            b
        }
    }
}

bitflags! {
    pub struct WriteFrameFlags: u8 {
        /// Bit 0 (LSB) = if 1, start output immediately, instead of waiting for current frame (if there is one) to finish playing
        const StartImmediately  = 0b0000_0001;
        /// Bit 1 = if 1, play frame only once, instead of repeating until another frame is written
        const SingleMode        = 0b0000_0010;
        /// Bit 2 = if 1, don't let WriteFrame() block execution while waiting for the transfer to finish
        const DontBlock         = 0b0000_0100;
    }
}

