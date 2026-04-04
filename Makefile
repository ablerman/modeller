.PHONY: run run-release build build-release test clean

run:
	~/.cargo/bin/cargo run -p brep-app

run-release:
	~/.cargo/bin/cargo run -p brep-app --release

build:
	~/.cargo/bin/cargo build --workspace

build-release:
	~/.cargo/bin/cargo build --workspace --release

test:
	~/.cargo/bin/cargo test --workspace --exclude brep-app

clean:
	~/.cargo/bin/cargo clean
