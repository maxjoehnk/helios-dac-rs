use helios_dac::{Frame, Point, Coordinate, Color};

#[cfg(all(feature = "sdk", not(feature = "native")))]
pub fn main() {
    use helios_dac::wrapper::HeliosDacWrapper;

    let frames = get_frames();

    let mut controller = HeliosDacWrapper::new();
    let device_count = controller.open_devices().unwrap();

    for i in 0..device_count {
        for frame in frames.clone() {
            controller.write_frame(i, frame).unwrap();
        }

        controller.stop(i).unwrap();
    }
}

#[cfg(feature = "native")]
pub fn main() {
    use helios_dac::NativeHeliosDacController;

    let frames = get_frames();

    let controller = NativeHeliosDacController::new().unwrap();
    let devices = controller.list_devices().unwrap();

    for device in devices {
        let mut device = device.open().unwrap();

        for frame in frames.clone() {
            println!("status: {:?}", device.status().unwrap());
            device.write_frame(frame).unwrap();
        }

        device.stop().unwrap();
    }
}

fn get_frames() -> Vec<Frame> {
    let mut frames = vec![];

    let color = Color::new(0xD0, 0xFF, 0xD0);

    for i in 0..30 {
        let mut points = vec![];
        let mut y = (i * 0xFFF / 30) as u16;
        for j in 0..1000 {
            let x = if j < 500 {
                j * 0xFFF / 500
            }else {
                0xFFF - ((j - 500) * 0xFFF / 500)
            } as u16;

            points.push(Point {
                coordinate: (x, y).into(),
                color,
                intensity: 0xFF
            });
        }

        frames.push(Frame::new(30000, points));
    }

    frames
}
