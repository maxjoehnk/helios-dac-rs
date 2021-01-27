# helios-dac-rs

A Rust library to interact with the [Helios Laser DAC](https://bitlasers.com/helios-laser-dac/).

**DISCLAIMER: I have not properly tested this with a real laser.
Be careful as Lasers may harm your eyes when used improperly.**

Wrapper for the official [sdk](https://github.com/Grix/helios_dac) as well as a native rust implementation of the protocol.

*Disclaimer:* Because of compilation issues on my main machine I could not test the sdk version against the dac.
I've made sure it compiles, but my focus is on the rust native version.

## Usage

For now you have to reference this crate from git like so:

```toml
[dependencies]
helios-dac = { git = "https://github.com/maxjoehnk/helios-dac-rs.git" }
```

To use the rust version instead of the offical sdk use the `native` feature and disable default features:

```toml
[dependencies]
helios-dac = { git = "https://github.com/maxjoehnk/helios-dac-rs.git", default-features = false, features = ["native"] }
```

## Development

To build the sdk version you can use the Makefile.
It will spin up a docker container with the required build dependencies.
