FROM rust:latest

RUN apt update && apt install -y llvm-dev libclang-dev clang libusb-1.0-0-dev libusb-1.0-0

WORKDIR /src

CMD cargo build
