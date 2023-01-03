use std::path::PathBuf;

fn main() {
    generate_bindings();
    build_sdk();
}

fn generate_bindings() {
    println!("cargo:rerun-if-changed=wrapper.hpp");
    println!("cargo:rerun-if-changed=sdk/sdk/HeliosDac.h");
    println!("cargo:rerun-if-changed=sdk/sdk/HeliosDacAPI.h");
    pkg_config::find_library("libusb-1.0").expect("could not find libusb to link to");
    let bindings = bindgen::builder()
        .header("wrapper.hpp")
        .generate_comments(true)
        .clang_arg("-std=c++14")
        .opaque_type("std::.*")
        .opaque_type("libusb_.*")
        .allowlist_type("HeliosDac")
        .allowlist_var("HELIOS_SUCCESS")
        .allowlist_var("HELIOS_ERROR_.*")
        .parse_callbacks(Box::new(bindgen::CargoCallbacks))
        .generate()
        .expect("Unable to generate bindings");
    let out_path = PathBuf::from(std::env::var("OUT_DIR").unwrap());
    bindings
        .write_to_file(out_path.join("bindings.rs"))
        .expect("Couldn't write bindings!");
}

fn build_sdk() {
    cc::Build::new()
        .cpp(true)
        .flag("-std=c++14")
        .flag("-fPIC")
        .opt_level(2)
        .files(&["sdk/sdk/HeliosDacAPI.cpp", "sdk/sdk/HeliosDac.cpp"])
        .shared_flag(true)
        .compile("HeliosDacAPI");
}
