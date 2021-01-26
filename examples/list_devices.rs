use helios_dac::wrapper::HeliosDacWrapper;

pub fn main() {
    let mut controller = HeliosDacWrapper::new();
    let device_count = controller.open_devices().unwrap();

    for i in 0..device_count {
        let name = controller.name(i).unwrap();
        let status = controller.status(i);

        println!("{}: {:?}", name, status);
    }
}
