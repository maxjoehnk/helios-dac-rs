USER = 1000

build: base-image
	docker run --rm -v $(PWD):/src -u $(USER) helios-build cargo build --examples

doc: base-image
	docker run --rm -v $(PWD):/src -u $(USER) helios-build cargo doc

base-image: Dockerfile
	docker build . -t helios-build

examples: list-devices

list-devices:
	docker run --rm -v $(PWD):/src -u $(USER) --privileged helios-build cargo run --example list_devices
