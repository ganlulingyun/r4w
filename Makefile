# R4W - Rust for Waveforms - Makefile
# ===================================
#
# R4W is a platform for developing, testing, and deploying SDR waveforms in Rust.
# It provides a foundation of common libraries for porting existing waveforms
# and developing new ones, leveraging Rust's safety, performance, and cross-compilation.

.PHONY: all build release run run-release web web-release serve test test-all \
        check clippy fmt clean clean-all doc install-deps help \
        bench bench-core bench-dsp bench-parallel \
        arm-install arm-targets build-arm64 build-arm32 build-core-arm64 build-core-arm32 \
        bench-build-arm64 bench-build-arm32 cross-arm64 cross-arm32 \
        remote-bench remote-bench-arm64 remote-bench-arm32 remote-deploy-arm64 \
        remote-run remote-run-group remote-clean remote-help \
        install install-system uninstall uninstall-system \
        build-agent deploy-arm64 deploy-both remote-agent-start remote-agent-stop \
        remote-agent-status remote-agent-logs deploy-and-start deploy-help \
        slides slide slides-demo slides-images slides-clean slides-list

# Default target
all: build

# ============================================================================
# Build Targets
# ============================================================================

## Build all crates (debug)
build:
	cargo build

## Build all crates (release, optimized)
release:
	cargo build --release

## Build only the GUI crate
build-gui:
	cargo build -p r4w-gui

## Build only the CLI crate
build-cli:
	cargo build -p r4w-cli

## Build only the core DSP library
build-core:
	cargo build -p r4w-core

# ============================================================================
# Run Targets
# ============================================================================

## Run the GUI application (debug)
run:
	cargo run --bin r4w-explorer

## Run the GUI application (release)
run-release:
	cargo run --release --bin r4w-explorer

## Run the CLI with arguments (use: make cli ARGS="info --sf 7")
cli:
	cargo run --bin r4w -- $(ARGS)

## Show CLI help
cli-help:
	cargo run --bin r4w -- --help

## Simulate a LoRa transmission
simulate:
	cargo run --bin r4w -- simulate --message "Hello R4W!" --snr 10.0

## Show LoRa parameter info
info:
	cargo run --bin r4w -- info --sf 7 --bw 125

## List available waveforms
waveforms:
	cargo run --bin r4w -- waveform --list

# ============================================================================
# Web/WASM Targets
# ============================================================================

## Build and serve the web application (development)
web serve:
	cd crates/r4w-web && trunk serve --port 8089

## Build the web application (release)
web-release:
	cd crates/r4w-web && trunk build --release

## Build web application only (no serve)
web-build:
	cd crates/r4w-web && trunk build

## Clean web build artifacts
web-clean:
	rm -rf crates/r4w-web/dist

# ============================================================================
# Test Targets
# ============================================================================

## Run all tests
test:
	cargo test

## Run tests with output shown
test-verbose:
	cargo test -- --nocapture

## Run tests for a specific crate (use: make test-crate CRATE=r4w-core)
test-crate:
	cargo test -p $(CRATE)

## Run only r4w-core tests
test-core:
	cargo test -p r4w-core

## Run only r4w-sim tests
test-sim:
	cargo test -p r4w-sim

## Run only r4w-gui tests
test-gui:
	cargo test -p r4w-gui

## Run doc tests
test-doc:
	cargo test --doc

## Run all tests including ignored ones
test-all:
	cargo test -- --include-ignored

# ============================================================================
# Code Quality
# ============================================================================

## Check code compiles without building
check:
	cargo check --all-targets

## Run clippy linter
clippy:
	cargo clippy --all-targets -- -D warnings

## Run clippy with suggestions
clippy-fix:
	cargo clippy --all-targets --fix --allow-dirty

## Format code
fmt:
	cargo fmt

## Check formatting without changes
fmt-check:
	cargo fmt -- --check

## Run all quality checks (check, clippy, fmt-check, test)
quality: check clippy fmt-check test

# ============================================================================
# Presentations / Slides
# ============================================================================

# Presentation source and output directories
SLIDES_SRC := docs/presentations
SLIDES_OUT := docs/slides

# Presentation files (markdown sources)
PRESENTATIONS := 01_platform_overview 02_technical_deep_dive 03_workshop_intro \
                 04_why_rust_for_sdr 05_r4w_demo 06_fpga_acceleration \
                 07_security_isolation 08_waveform_development 09_realtime_systems \
                 10_workshop_series

## Build all HTML slideshows from markdown presentations
slides: slides-images $(PRESENTATIONS:%=$(SLIDES_OUT)/%.html)
	@echo "All slides built in $(SLIDES_OUT)/"

## Build HTML documentation from markdown
docs-html:
	@mkdir -p $(SLIDES_OUT)/docs $(SLIDES_OUT)/docs/porting
	@echo "Converting documentation to HTML..."
	@for doc in README.md:readme OVERVIEW.md:overview \
		docs/WAVEFORM_DEVELOPERS_GUIDE.md:waveform-guide \
		docs/FPGA_DEVELOPERS_GUIDE.md:fpga-guide \
		docs/SECURITY_GUIDE.md:security-guide \
		docs/ISOLATION_GUIDE.md:isolation-guide \
		docs/REALTIME_SCHEDULER_GUIDE.md:realtime-guide \
		docs/PHYSICAL_LAYER_GUIDE.md:physical-layer-guide \
		docs/PORTING_GUIDE_MILITARY.md:porting-military \
		docs/CRYPTO_BOUNDARY.md:crypto-boundary \
		docs/TICK_SCHEDULER_GUIDE.md:tick-scheduler; do \
		src=$${doc%%:*}; name=$${doc##*:}; \
		pandoc "$$src" -o "$(SLIDES_OUT)/docs/$$name.html" --standalone \
			--css="style.css" --metadata title="R4W - $$name"; \
	done
	@for f in docs/porting/*.md; do \
		name=$$(basename "$$f" .md); \
		[ "$$name" != "README" ] && pandoc "$$f" -o "$(SLIDES_OUT)/docs/porting/$$name.html" \
			--standalone --css="../style.css" --metadata title="Porting: $$name"; \
	done || true
	@echo "Documentation built in $(SLIDES_OUT)/docs/"

## Build a single slideshow (use: make slide NAME=05_r4w_demo)
slide: slides-images
	@mkdir -p $(SLIDES_OUT)
	pandoc -t revealjs -s $(SLIDES_SRC)/$(NAME).md \
		-o $(SLIDES_OUT)/$(NAME).html \
		-V revealjs-url=https://unpkg.com/reveal.js@5.1.0 \
		-V theme=night \
		-V transition=slide \
		-V slideNumber=true \
		-V hash=true \
		-V width=1920 \
		-V height=1080
	@echo "Built: $(SLIDES_OUT)/$(NAME).html"

## Pattern rule for building individual slides
$(SLIDES_OUT)/%.html: $(SLIDES_SRC)/%.md
	@mkdir -p $(SLIDES_OUT)
	pandoc -t revealjs -s $< -o $@ \
		-V revealjs-url=https://unpkg.com/reveal.js@5.1.0 \
		-V theme=night \
		-V transition=slide \
		-V slideNumber=true \
		-V hash=true \
		-V width=1920 \
		-V height=1080
	@echo "Built: $@"

## Build slides and open the demo presentation
slides-demo: $(SLIDES_OUT)/05_r4w_demo.html
	@echo "Opening demo slides..."
	@xdg-open $(SLIDES_OUT)/05_r4w_demo.html 2>/dev/null || open $(SLIDES_OUT)/05_r4w_demo.html 2>/dev/null || echo "Open $(SLIDES_OUT)/05_r4w_demo.html in your browser"

## Copy images to slides directory for presentations
slides-images:
	@mkdir -p $(SLIDES_OUT)/images/screenshots
	@cp -r images/screenshots/*.png $(SLIDES_OUT)/images/screenshots/ 2>/dev/null || true
	@cp images/*.png $(SLIDES_OUT)/images/ 2>/dev/null || true
	@echo "Images copied to $(SLIDES_OUT)/images/"

## Clean generated slides
slides-clean:
	rm -rf $(SLIDES_OUT)
	@echo "Cleaned slides output directory"

## List available presentations
slides-list:
	@echo "Available presentations:"
	@for name in $(PRESENTATIONS); do \
		echo "  $$name"; \
	done
	@echo ""
	@echo "Build all: make slides"
	@echo "Build one: make slide NAME=05_r4w_demo"
	@echo "View demo: make slides-demo"

# ============================================================================
# Documentation
# ============================================================================

## Generate documentation
doc:
	cargo doc --no-deps

## Generate and open documentation
doc-open:
	cargo doc --no-deps --open

## Generate documentation for all dependencies too
doc-all:
	cargo doc --open

# ============================================================================
# Benchmarks
# ============================================================================

## Run benchmarks
bench:
	cargo bench

## Run benchmarks for r4w-core only
bench-core:
	cargo bench -p r4w-core

## Run benchmarks for r4w-core DSP operations
bench-dsp:
	cargo bench -p r4w-core --bench dsp_bench

## Run parallel benchmarks (requires parallel feature)
bench-parallel:
	cargo bench -p r4w-core --features parallel --bench parallel_bench

# ============================================================================
# ARM Cross-Compilation
# ============================================================================

# ARM64 target (Raspberry Pi 4, Apple M1/M2, etc.)
ARM64_TARGET := aarch64-unknown-linux-gnu
# ARM32 target (Raspberry Pi 3 and earlier, 32-bit ARM devices)
ARM32_TARGET := armv7-unknown-linux-gnueabihf

## Install ARM cross-compilation toolchains
arm-install:
	rustup target add $(ARM64_TARGET)
	rustup target add $(ARM32_TARGET)
	@echo ""
	@echo "ARM targets installed. You also need cross-compilation toolchain:"
	@echo "  Ubuntu/Debian: sudo apt install gcc-aarch64-linux-gnu gcc-arm-linux-gnueabihf"
	@echo "  Fedora: sudo dnf install gcc-aarch64-linux-gnu gcc-arm-linux-gnueabihf"
	@echo "  macOS: brew tap messense/macos-cross-toolchains && brew install aarch64-unknown-linux-gnu"
	@echo ""
	@echo "Or use 'cross' for easier Docker-based cross-compilation:"
	@echo "  cargo install cross"

## Build for ARM64 (aarch64-unknown-linux-gnu) - all crates
build-arm64:
	cargo build --release --target $(ARM64_TARGET)

## Build for ARM32 (armv7-unknown-linux-gnueabihf) - all crates
build-arm32:
	cargo build --release --target $(ARM32_TARGET)

## Build r4w CLI only for ARM64 (for Raspberry Pi agent deployment)
## Uses 'cross' for Docker-based cross-compilation
build-cli-arm64:
	cross build --release --target $(ARM64_TARGET) -p r4w-cli

## Build r4w CLI only for ARM32
## Uses 'cross' for Docker-based cross-compilation
build-cli-arm32:
	cross build --release --target $(ARM32_TARGET) -p r4w-cli

## Build r4w-core only for ARM64
build-core-arm64:
	cargo build --release --target $(ARM64_TARGET) -p r4w-core

## Build r4w-core only for ARM32
build-core-arm32:
	cargo build --release --target $(ARM32_TARGET) -p r4w-core

## Build benchmarks for ARM64 (produces benchmark binary for remote execution)
bench-build-arm64:
	cargo build --release --target $(ARM64_TARGET) -p r4w-core --bench dsp_bench
	@echo ""
	@echo "Benchmark binary built at:"
	@echo "  target/$(ARM64_TARGET)/release/deps/dsp_bench-*"
	@echo ""
	@echo "Copy to ARM device and run with:"
	@echo "  scp target/$(ARM64_TARGET)/release/deps/dsp_bench-* user@arm-host:~/"
	@echo "  ssh user@arm-host './dsp_bench-* --bench'"

## Build benchmarks for ARM32
bench-build-arm32:
	cargo build --release --target $(ARM32_TARGET) -p r4w-core --bench dsp_bench
	@echo ""
	@echo "Benchmark binary built at:"
	@echo "  target/$(ARM32_TARGET)/release/deps/dsp_bench-*"

## Cross-compile using Docker (requires 'cross' installed)
cross-arm64:
	cross build --release --target $(ARM64_TARGET)

cross-arm32:
	cross build --release --target $(ARM32_TARGET)

## Show ARM build targets
arm-targets:
	@echo "Available ARM targets:"
	@echo "  $(ARM64_TARGET) - 64-bit ARM (Raspberry Pi 4, M1/M2, AWS Graviton)"
	@echo "  $(ARM32_TARGET) - 32-bit ARM (Raspberry Pi 2/3, older devices)"
	@echo ""
	@echo "Use 'make arm-install' to install targets"
	@echo "Use 'make build-arm64' or 'make build-arm32' to build"

# ============================================================================
# Remote ARM Benchmarking
# ============================================================================

# Remote host configuration (override with: make remote-bench ARM_HOST=user@pi.local)
ARM_HOST ?= pi@raspberrypi.local
ARM_DIR ?= /tmp/r4w-bench
BENCH_NAME ?= dsp_bench

## Deploy and run benchmarks on remote ARM64 host
remote-bench: remote-bench-arm64

## Build, deploy, and run benchmarks on ARM64 host
remote-bench-arm64: bench-build-arm64
	@echo "Deploying benchmark to $(ARM_HOST):$(ARM_DIR)"
	ssh $(ARM_HOST) "mkdir -p $(ARM_DIR)"
	scp target/$(ARM64_TARGET)/release/deps/$(BENCH_NAME)-* $(ARM_HOST):$(ARM_DIR)/$(BENCH_NAME)
	@echo ""
	@echo "Running benchmark on $(ARM_HOST)..."
	@echo "============================================"
	ssh $(ARM_HOST) "cd $(ARM_DIR) && ./$(BENCH_NAME) --bench"

## Build, deploy, and run benchmarks on ARM32 host
remote-bench-arm32: bench-build-arm32
	@echo "Deploying benchmark to $(ARM_HOST):$(ARM_DIR)"
	ssh $(ARM_HOST) "mkdir -p $(ARM_DIR)"
	scp target/$(ARM32_TARGET)/release/deps/$(BENCH_NAME)-* $(ARM_HOST):$(ARM_DIR)/$(BENCH_NAME)
	@echo ""
	@echo "Running benchmark on $(ARM_HOST)..."
	@echo "============================================"
	ssh $(ARM_HOST) "cd $(ARM_DIR) && ./$(BENCH_NAME) --bench"

## Deploy benchmark binary only (without running)
remote-deploy-arm64: bench-build-arm64
	@echo "Deploying benchmark to $(ARM_HOST):$(ARM_DIR)"
	ssh $(ARM_HOST) "mkdir -p $(ARM_DIR)"
	scp target/$(ARM64_TARGET)/release/deps/$(BENCH_NAME)-* $(ARM_HOST):$(ARM_DIR)/$(BENCH_NAME)
	@echo "Deployed! Run with: ssh $(ARM_HOST) '$(ARM_DIR)/$(BENCH_NAME) --bench'"

## Run benchmark on remote host (assumes already deployed)
remote-run:
	ssh $(ARM_HOST) "cd $(ARM_DIR) && ./$(BENCH_NAME) --bench"

## Run specific benchmark group on remote host
remote-run-group:
	@echo "Running benchmark group '$(GROUP)' on $(ARM_HOST)"
	ssh $(ARM_HOST) "cd $(ARM_DIR) && ./$(BENCH_NAME) --bench '$(GROUP)'"

## Clean remote benchmark directory
remote-clean:
	ssh $(ARM_HOST) "rm -rf $(ARM_DIR)"
	@echo "Cleaned remote benchmark directory"

## Show remote benchmark help
remote-help:
	@echo "Remote ARM Benchmarking"
	@echo ""
	@echo "Configuration (set via environment or make arguments):"
	@echo "  ARM_HOST   - SSH host (default: pi@raspberrypi.local)"
	@echo "  ARM_DIR    - Remote directory (default: /tmp/r4w-bench)"
	@echo "  BENCH_NAME - Benchmark name (default: dsp_bench)"
	@echo ""
	@echo "Examples:"
	@echo "  make remote-bench ARM_HOST=user@pi4.local"
	@echo "  make remote-bench-arm32 ARM_HOST=user@pi3.local"
	@echo "  make remote-run-group GROUP=fft ARM_HOST=user@pi.local"
	@echo ""
	@echo "Workflow:"
	@echo "  1. Ensure SSH key authentication is set up"
	@echo "  2. Run 'make remote-bench ARM_HOST=user@host'"
	@echo "  3. Results are displayed on stdout"
	@echo ""
	@echo "Available benchmark groups:"
	@echo "  chirp_generation, chirp_types, lora_modulation, lora_demodulation,"
	@echo "  fft, fft_power_spectrum, spectrogram, waveform_modulation,"
	@echo "  waveform_demodulation, waveform_roundtrip, coding, complex_ops,"
	@echo "  peak_finding, simd_utils"

# ============================================================================
# Installation
# ============================================================================

# Installation directories
PREFIX ?= $(HOME)/.local
BINDIR ?= $(PREFIX)/bin
SYSTEM_PREFIX ?= /usr/local
SYSTEM_BINDIR ?= $(SYSTEM_PREFIX)/bin

# Binary names
BINARIES := r4w-explorer r4w

## Install to ~/.local/bin (user-local, no sudo required)
install: release
	@echo "Installing to $(BINDIR)..."
	@mkdir -p $(BINDIR)
	@cp target/release/r4w-explorer $(BINDIR)/
	@cp target/release/r4w $(BINDIR)/
	@echo ""
	@echo "Installed:"
	@echo "  $(BINDIR)/r4w-explorer  - GUI application"
	@echo "  $(BINDIR)/r4w           - Command-line interface"
	@echo ""
	@if ! echo "$$PATH" | grep -q "$(BINDIR)"; then \
		echo "NOTE: Add $(BINDIR) to your PATH:"; \
		echo "  export PATH=\"$(BINDIR):\$$PATH\""; \
		echo ""; \
	fi
	@echo "Run 'r4w-explorer' to start the GUI"
	@echo "Run 'r4w --help' for CLI usage"

## Install to /usr/local/bin (system-wide, requires sudo)
install-system: release
	@echo "Installing to $(SYSTEM_BINDIR) (requires sudo)..."
	sudo mkdir -p $(SYSTEM_BINDIR)
	sudo cp target/release/r4w-explorer $(SYSTEM_BINDIR)/
	sudo cp target/release/r4w $(SYSTEM_BINDIR)/
	@echo ""
	@echo "Installed system-wide:"
	@echo "  $(SYSTEM_BINDIR)/r4w-explorer"
	@echo "  $(SYSTEM_BINDIR)/r4w"

## Uninstall from ~/.local/bin
uninstall:
	@echo "Removing from $(BINDIR)..."
	@rm -f $(BINDIR)/r4w-explorer
	@rm -f $(BINDIR)/r4w
	@echo "Uninstalled"

## Uninstall from /usr/local/bin (requires sudo)
uninstall-system:
	@echo "Removing from $(SYSTEM_BINDIR) (requires sudo)..."
	sudo rm -f $(SYSTEM_BINDIR)/r4w-explorer
	sudo rm -f $(SYSTEM_BINDIR)/r4w
	@echo "Uninstalled"

# ============================================================================
# Remote Agent Deployment (Raspberry Pi)
# ============================================================================

# Remote deployment configuration
REMOTE_HOST ?= pi@raspberrypi.local
REMOTE_USER ?= $(word 1, $(subst @, ,$(REMOTE_HOST)))
REMOTE_DIR ?= ~/.local/bin

## Build r4w CLI for ARM64 (for Raspberry Pi deployment)
build-agent: build-cli-arm64

## Deploy r4w agent to remote Raspberry Pi
deploy-arm64: build-cli-arm64
	@echo "Deploying r4w to $(REMOTE_HOST):$(REMOTE_DIR)"
	ssh $(REMOTE_HOST) "mkdir -p $(REMOTE_DIR)"
	scp target/$(ARM64_TARGET)/release/r4w $(REMOTE_HOST):$(REMOTE_DIR)/
	@echo ""
	@echo "Deployed! Start agent with:"
	@echo "  ssh $(REMOTE_HOST) 'r4w agent --port 6000'"
	@echo ""
	@echo "Or run in background:"
	@echo "  ssh $(REMOTE_HOST) 'nohup r4w agent --port 6000 > /tmp/r4w-agent.log 2>&1 &'"

## Deploy to both TX and RX Raspberry Pis
deploy-both: build-cli-arm64
	@echo "Deploying to TX host: $(TX_HOST)"
	ssh $(TX_HOST) "mkdir -p $(REMOTE_DIR)"
	scp target/$(ARM64_TARGET)/release/r4w $(TX_HOST):$(REMOTE_DIR)/
	@echo "Deploying to RX host: $(RX_HOST)"
	ssh $(RX_HOST) "mkdir -p $(REMOTE_DIR)"
	scp target/$(ARM64_TARGET)/release/r4w $(RX_HOST):$(REMOTE_DIR)/
	@echo ""
	@echo "Both deployed! Start agents with:"
	@echo "  TX: ssh $(TX_HOST) 'r4w agent'"
	@echo "  RX: ssh $(RX_HOST) 'r4w agent'"

## Start agent on remote host (assumes r4w is deployed)
remote-agent-start:
	@echo "Starting agent on $(REMOTE_HOST)..."
	ssh $(REMOTE_HOST) "nohup $(REMOTE_DIR)/r4w agent --port 6000 > /tmp/r4w-agent.log 2>&1 &"
	@sleep 1
	@echo "Agent started. Check status with: r4w remote -a $(REMOTE_HOST) status"

## Stop agent on remote host
remote-agent-stop:
	@echo "Stopping agent on $(REMOTE_HOST)..."
	ssh $(REMOTE_HOST) "pkill -f 'r4w agent' || true"
	@echo "Agent stopped"

## Check agent status on remote host
remote-agent-status:
	@echo "Checking agent on $(REMOTE_HOST)..."
	@ssh $(REMOTE_HOST) "pgrep -a 'r4w' || echo 'No agent running'"

## View agent logs on remote host
remote-agent-logs:
	ssh $(REMOTE_HOST) "tail -50 /tmp/r4w-agent.log 2>/dev/null || echo 'No log file'"

## Quick setup: Deploy and start agent
deploy-and-start: deploy-arm64 remote-agent-start
	@echo ""
	@echo "Agent deployed and started on $(REMOTE_HOST)"

## Show deployment help
deploy-help:
	@echo "Remote Agent Deployment"
	@echo "======================"
	@echo ""
	@echo "Configuration variables:"
	@echo "  REMOTE_HOST - SSH host (default: pi@raspberrypi.local)"
	@echo "  REMOTE_DIR  - Install directory (default: ~/.local/bin)"
	@echo "  TX_HOST     - TX Raspberry Pi (for deploy-both)"
	@echo "  RX_HOST     - RX Raspberry Pi (for deploy-both)"
	@echo ""
	@echo "Quick start:"
	@echo "  make deploy-and-start REMOTE_HOST=joe@192.168.179.104"
	@echo ""
	@echo "Deploy to both TX and RX:"
	@echo "  make deploy-both TX_HOST=joe@192.168.179.104 RX_HOST=rpi@192.168.179.129"
	@echo ""
	@echo "After deployment, use the GUI 'Remote Lab' view or CLI:"
	@echo "  r4w remote -a 192.168.179.104 status"
	@echo "  r4w remote -a 192.168.179.104 start-tx -t 192.168.179.129:5000 -w BPSK"
	@echo "  r4w remote -a 192.168.179.129 start-rx -w BPSK -p 5000"

# ============================================================================
# Clean Targets
# ============================================================================

## Clean build artifacts
clean:
	cargo clean

## Clean everything including web artifacts
clean-all: clean web-clean
	rm -rf target/

# ============================================================================
# Dependencies
# ============================================================================

## Install required tools for development
install-deps:
	rustup target add wasm32-unknown-unknown
	cargo install trunk
	@echo "All dependencies installed!"

## Update dependencies
update:
	cargo update

## Show outdated dependencies
outdated:
	cargo outdated || echo "Install cargo-outdated: cargo install cargo-outdated"

# ============================================================================
# Development Helpers
# ============================================================================

## Watch for changes and rebuild
watch:
	cargo watch -x build || echo "Install cargo-watch: cargo install cargo-watch"

## Watch for changes and run tests
watch-test:
	cargo watch -x test || echo "Install cargo-watch: cargo install cargo-watch"

## Count lines of code
loc:
	@echo "Lines of Rust code:"
	@find crates -name "*.rs" | xargs wc -l | tail -1

## Show project structure
tree:
	@tree -I 'target|dist|*.wasm|*.js' -L 3 crates/

# ============================================================================
# Help
# ============================================================================

## Show this help message
help:
	@echo "R4W - Rust for Waveforms"
	@echo "========================"
	@echo "A platform for developing, testing, and deploying SDR waveforms in Rust."
	@echo ""
	@echo "Build:"
	@echo "  make build        - Build all crates (debug)"
	@echo "  make release      - Build all crates (release)"
	@echo "  make build-gui    - Build only GUI crate"
	@echo "  make build-cli    - Build only CLI crate"
	@echo ""
	@echo "Run:"
	@echo "  make run          - Run GUI (debug)"
	@echo "  make run-release  - Run GUI (release)"
	@echo "  make cli ARGS=... - Run CLI with arguments"
	@echo "  make simulate     - Run LoRa simulation"
	@echo "  make waveforms    - List available waveforms"
	@echo ""
	@echo "Web/WASM:"
	@echo "  make web/serve    - Build and serve web app (port 8089)"
	@echo "  make web-release  - Build web app (release)"
	@echo "  make web-clean    - Clean web build artifacts"
	@echo ""
	@echo "Test:"
	@echo "  make test         - Run all tests"
	@echo "  make test-verbose - Run tests with output"
	@echo "  make test-core    - Run r4w-core tests"
	@echo "  make test-all     - Run all tests including ignored"
	@echo ""
	@echo "Benchmarks:"
	@echo "  make bench        - Run all benchmarks"
	@echo "  make bench-core   - Run r4w-core benchmarks"
	@echo "  make bench-dsp    - Run DSP benchmarks"
	@echo "  make bench-parallel - Run parallel benchmarks"
	@echo ""
	@echo "ARM Cross-Compilation:"
	@echo "  make arm-install  - Install ARM targets"
	@echo "  make arm-targets  - Show available ARM targets"
	@echo "  make build-arm64  - Build for ARM64 (Pi 4, M1)"
	@echo "  make build-arm32  - Build for ARM32 (Pi 2/3)"
	@echo "  make bench-build-arm64 - Build benchmarks for ARM64"
	@echo "  make cross-arm64  - Cross-compile with Docker"
	@echo ""
	@echo "Remote Benchmarking:"
	@echo "  make remote-bench ARM_HOST=user@host - Full remote benchmark"
	@echo "  make remote-deploy-arm64 - Deploy only"
	@echo "  make remote-run   - Run deployed benchmark"
	@echo "  make remote-help  - Show detailed remote options"
	@echo ""
	@echo "Remote Agent (Raspberry Pi):"
	@echo "  make deploy-arm64 REMOTE_HOST=user@host - Deploy agent to RPi"
	@echo "  make deploy-both TX_HOST=... RX_HOST=... - Deploy to TX/RX pair"
	@echo "  make deploy-and-start REMOTE_HOST=... - Deploy and start agent"
	@echo "  make remote-agent-start - Start agent on remote"
	@echo "  make remote-agent-stop  - Stop agent on remote"
	@echo "  make deploy-help  - Show detailed deployment options"
	@echo ""
	@echo "Quality:"
	@echo "  make check        - Check compilation"
	@echo "  make clippy       - Run clippy linter"
	@echo "  make fmt          - Format code"
	@echo "  make quality      - Run all quality checks"
	@echo ""
	@echo "Docs:"
	@echo "  make doc          - Generate documentation"
	@echo "  make doc-open     - Generate and open docs"
	@echo ""
	@echo "Presentations:"
	@echo "  make slides       - Build all HTML slideshows"
	@echo "  make slides-demo  - Build and open demo slideshow"
	@echo "  make slides-list  - List available presentations"
	@echo "  make slide NAME=X - Build single presentation"
	@echo "  make slides-clean - Remove generated slides"
	@echo ""
	@echo "Install:"
	@echo "  make install      - Install to ~/.local/bin"
	@echo "  make install-system - Install to /usr/local/bin (sudo)"
	@echo "  make uninstall    - Uninstall from ~/.local/bin"
	@echo "  make uninstall-system - Uninstall from /usr/local/bin"
	@echo ""
	@echo "Other:"
	@echo "  make clean        - Clean build artifacts"
	@echo "  make clean-all    - Clean everything"
	@echo "  make install-deps - Install required tools"
	@echo "  make help         - Show this help"
