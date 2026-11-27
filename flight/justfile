# flight: PX4 SITL / Gazebo / QGroundControl sim. `just --list` for recipes.

image := env_var_or_default("FLIGHT_PX4_IMAGE", "flight-px4-gz:latest")

# Run the sim (web menu); extra args go to flightsim.
run *args:
    cargo run --release -p flightsim --bin flightsim -- {{args}}

# Unit tests and clippy for the whole workspace.
test:
    cargo test --workspace
    cargo clippy --workspace --all-targets -- -D warnings

# Tests that launch real PX4 (needs FLIGHT_PX4_DIR or the image).
test-px4:
    cargo test -p flight-launch -- --ignored --nocapture

# Build the PX4 SITL + Gazebo Harmonic image (docker/px4-gz.Dockerfile).
px4-image *args:
    docker build -f docker/px4-gz.Dockerfile -t {{image}} {{args}} docker

# QGroundControl: the one on PATH (nix develop), else nixpkgs'.
qgc:
    if command -v QGroundControl >/dev/null; then exec QGroundControl; else exec nix run .#qgc; fi

# Check that the flake evaluates.
flake-check:
    nix flake check --no-build
