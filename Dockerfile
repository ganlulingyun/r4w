# R4W - Rust for Waveforms
# Multi-stage build for minimal runtime image

# Build stage
FROM docker.io/library/rust:1.85-slim-bookworm AS builder

# Install build dependencies
RUN apt-get update && apt-get install -y \
    pkg-config \
    libssl-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /build

# Copy workspace manifests first for dependency caching
COPY Cargo.toml Cargo.lock ./

# Copy all crate manifests
COPY crates/r4w-core/Cargo.toml crates/r4w-core/
COPY crates/r4w-sim/Cargo.toml crates/r4w-sim/
COPY crates/r4w-cli/Cargo.toml crates/r4w-cli/
COPY crates/r4w-fpga/Cargo.toml crates/r4w-fpga/
COPY crates/r4w-gui/Cargo.toml crates/r4w-gui/
COPY crates/r4w-web/Cargo.toml crates/r4w-web/
COPY crates/r4w-sandbox/Cargo.toml crates/r4w-sandbox/
COPY crates/r4w-ffi/Cargo.toml crates/r4w-ffi/
COPY workshop/Cargo.toml workshop/
COPY plugins/example-waveform/Cargo.toml plugins/example-waveform/

# Create dummy source files for dependency compilation
RUN mkdir -p crates/r4w-core/src crates/r4w-sim/src crates/r4w-cli/src \
    crates/r4w-fpga/src crates/r4w-gui/src crates/r4w-web/src \
    crates/r4w-sandbox/src crates/r4w-ffi/src workshop/src \
    plugins/example-waveform/src && \
    echo "fn main() {}" > crates/r4w-cli/src/main.rs && \
    echo "pub fn dummy() {}" > crates/r4w-core/src/lib.rs && \
    echo "pub fn dummy() {}" > crates/r4w-sim/src/lib.rs && \
    echo "pub fn dummy() {}" > crates/r4w-fpga/src/lib.rs && \
    echo "pub fn dummy() {}" > crates/r4w-gui/src/lib.rs && \
    echo "pub fn dummy() {}" > crates/r4w-web/src/lib.rs && \
    echo "pub fn dummy() {}" > crates/r4w-sandbox/src/lib.rs && \
    echo "pub fn dummy() {}" > crates/r4w-ffi/src/lib.rs && \
    echo "pub fn dummy() {}" > workshop/src/lib.rs && \
    echo "pub fn dummy() {}" > plugins/example-waveform/src/lib.rs

# Build dependencies only (this layer is cached)
RUN cargo build --release --bin r4w 2>/dev/null || true

# Copy actual source code
COPY crates/ crates/
COPY workshop/ workshop/
COPY plugins/ plugins/

# Touch source files to invalidate cache and rebuild
RUN touch crates/r4w-cli/src/main.rs

# Build release binary
RUN cargo build --release --bin r4w

# Runtime stage
FROM docker.io/library/debian:bookworm-slim AS runtime

# Install runtime dependencies
RUN apt-get update && apt-get install -y \
    ca-certificates \
    && rm -rf /var/lib/apt/lists/*

# Create non-root user
RUN useradd -m -u 1000 r4w

WORKDIR /app

# Copy binary from builder
COPY --from=builder /build/target/release/r4w /usr/local/bin/r4w

# Set ownership
RUN chown -R r4w:r4w /app

USER r4w

# Default command
ENTRYPOINT ["r4w"]
CMD ["--help"]

# Labels
LABEL org.opencontainers.image.title="R4W - Rust for Waveforms"
LABEL org.opencontainers.image.description="SDR waveform development platform"
LABEL org.opencontainers.image.source="https://github.com/joemooney/r4w"
LABEL org.opencontainers.image.licenses="MIT"
