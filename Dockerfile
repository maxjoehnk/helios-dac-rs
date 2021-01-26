FROM rust:latest

RUN apt update && apt install -y llvm-dev libclang-dev clang libusb-dev libusb-1.0

WORKDIR /src

CMD cargo build
