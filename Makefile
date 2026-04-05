.PHONY: run run-release build build-release test coverage clean

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

coverage:
	~/.cargo/bin/cargo llvm-cov --workspace --exclude brep-app --html
	@echo "Coverage report: target/llvm-cov/html/index.html"

clean:
	~/.cargo/bin/cargo clean
