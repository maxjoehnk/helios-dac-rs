#[cfg(all(feature = "sdk", not(feature = "native")))]
pub fn main() {
    use helios_dac::wrapper::HeliosDacWrapper;

    let mut controller = HeliosDacWrapper::new();
    let device_count = controller.open_devices().unwrap();

    for i in 0..device_count {
        let name = controller.name(i).unwrap();
        let status = controller.status(i);

        println!("{}: {:?}", name, status);
    }
}

#[cfg(feature = "native")]
pub fn main() {
    use helios_dac::NativeHeliosDacController;

    let controller = NativeHeliosDacController::new().unwrap();
    let devices = controller.list_devices().unwrap();

    for device in devices {
        let device = device.open().unwrap();
        let name = device.name().unwrap();
        let status = device.status().unwrap();
        let version = device.firmware_version().unwrap();

        println!("{:?} FW v{} - Status: {:?}", name, version, status);
    }
}
