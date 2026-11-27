{
  description = "flight: PX4 SITL / Gazebo / QGroundControl flight-control sim with runtime-swappable Rust stand-ins";

  # nixos-26.05 and rust-overlay, as in ~/src/middleware/flake.nix.
  #
  # What nix provides here: the Rust toolchain, the flightsim binary,
  # QGroundControl (nixpkgs `qgroundcontrol`, binary `QGroundControl`) and a
  # docker client. What it does NOT: Gazebo Harmonic and PX4 SITL. nixpkgs
  # has no gz-sim 8 (only pieces of the old Ignition stack) and no PX4
  # package; PX4's SITL build expects Ubuntu's OSRF packages and a writable
  # source tree. The supported routes are the container
  # (docker/px4-gz.Dockerfile, `nix run .#build-px4-image`) or a native
  # PX4-Autopilot checkout built with `make px4_sitl` and named by
  # FLIGHT_PX4_DIR. crates/flight-launch picks native if FLIGHT_PX4_DIR is
  # set, else the image.
  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-26.05";
    rust-overlay = {
      url = "github:oxalica/rust-overlay";
      inputs.nixpkgs.follows = "nixpkgs";
    };
    # The workspace depends on `middleware = { path = "../../middleware" }`
    # (this repo lives at ~/src/katas/flight). A flake build cannot reach
    # outside its own source, so middleware is an input and the build
    # assembles a tree where ../../middleware resolves (see `src` below).
    #
    # git+file rather than path: it copies only git-tracked files (not
    # middleware's multi-GB target/ or result symlinks). `ref=HEAD` makes
    # it the committed HEAD of whatever branch is checked out, so it locks
    # to a rev even while the middleware tree is dirty (without a ref nix
    # takes the dirty working tree, cannot lock it and refuses to write
    # flake.lock). Uncommitted middleware edits are therefore NOT in the
    # nix build (cargo builds see them). Refresh with
    # `nix flake update middleware`; point elsewhere with e.g.
    #   nix build .#flightsim --override-input middleware git+file:///path/to/middleware
    #   nix build .#flightsim --override-input middleware github:jvishnefske/middleware
    middleware = {
      url = "git+file:///home/j/src/middleware?ref=HEAD";
      flake = false;
    };
  };

  outputs = { self, nixpkgs, rust-overlay, middleware }:
    let
      system = "x86_64-linux";
      pkgs = import nixpkgs {
        inherit system;
        overlays = [ rust-overlay.overlays.default ];
      };
      inherit (pkgs) lib;

      toolchain = pkgs.rust-bin.stable.latest.default.override {
        extensions = [ "rust-src" "rust-analyzer" "clippy" "rustfmt" ];
      };
      rustPlatform = pkgs.makeRustPlatform { cargo = toolchain; rustc = toolchain; };

      # The cargo workspace only: no target/, nix files, docker context or
      # justfile, so editing those rebuilds nothing.
      flightSrc = lib.fileset.toSource {
        root = ./.;
        fileset = lib.fileset.difference ./. (lib.fileset.unions [
          (lib.fileset.maybeMissing ./target)
          (lib.fileset.maybeMissing ./docker)
          (lib.fileset.maybeMissing ./justfile)
          ./flake.nix
          (lib.fileset.maybeMissing ./flake.lock)
        ]);
      };

      # <src>/katas/flight and <src>/middleware, so the workspace's
      # `path = "../../middleware"` resolves exactly as in ~/src.
      src = pkgs.runCommand "flight-src" { } ''
        mkdir -p $out/katas
        cp -r ${flightSrc} $out/katas/flight
        cp -r ${middleware} $out/middleware
        chmod -R u+w $out
      '';

      version = (lib.importTOML ./Cargo.toml).workspace.package.version;

      # crates/flightsim (binary `flightsim`) is added later; until then
      # this evaluates but does not build.
      flightsim = rustPlatform.buildRustPackage {
        pname = "flightsim";
        inherit version src;
        sourceRoot = "flight-src/katas/flight";
        cargoLock.lockFile = ./Cargo.lock;
        cargoBuildFlags = [ "-p" "flightsim" "--bin" "flightsim" ];
        nativeBuildInputs = [ pkgs.pkg-config ];
        # The tests want UDP ports and, for the ignored ones, PX4.
        doCheck = false;
        meta = {
          description = "Flight-control sim: PX4/Gazebo/QGC or their Rust stand-ins, switched from a web menu";
          mainProgram = "flightsim";
          license = with lib.licenses; [ mit asl20 ];
        };
      };

      px4Image = "flight-px4-gz:latest";

      # Uses the host's docker (the daemon must match it anyway). The build
      # context is the docker/ directory copied to the store.
      buildPx4Image = pkgs.writeShellApplication {
        name = "build-px4-image";
        text = ''
          if ! command -v docker >/dev/null; then
            echo "build-px4-image: docker not found on PATH" >&2
            exit 1
          fi
          image="''${FLIGHT_PX4_IMAGE:-${px4Image}}"
          echo "building $image from docker/px4-gz.Dockerfile (PX4 + Gazebo Harmonic; takes a while)" >&2
          exec docker build \
            -f ${./docker}/px4-gz.Dockerfile \
            -t "$image" \
            "$@" \
            ${./docker}
        '';
      };

      qgc = pkgs.qgroundcontrol;
    in
    {
      packages.${system} = {
        inherit flightsim;
        default = flightsim;
        build-px4-image = buildPx4Image;
        inherit qgc;
      };

      apps.${system} = {
        default = { type = "app"; program = lib.getExe flightsim; meta.description = "Run flightsim"; };
        build-px4-image = {
          type = "app";
          program = lib.getExe buildPx4Image;
          meta.description = "docker build flight-px4-gz:latest (PX4 SITL + Gazebo Harmonic)";
        };
        qgc = { type = "app"; program = lib.getExe qgc; meta.description = "QGroundControl from nixpkgs"; };
      };

      devShells.${system}.default = pkgs.mkShell {
        packages = [
          toolchain
          pkgs.pkg-config
          pkgs.qgroundcontrol
          pkgs.docker-client
          pkgs.just
        ];
        # flight-launch finds QGroundControl on PATH; FLIGHT_QGC overrides.
        FLIGHT_PX4_IMAGE = px4Image;
        shellHook = ''
          echo "flight dev shell: cargo, clippy, rustfmt, rust-analyzer, just, QGroundControl, docker client"
          if [ -z "''${FLIGHT_PX4_DIR:-}" ]; then
            echo "  PX4/Gazebo: container $FLIGHT_PX4_IMAGE (build: just px4-image),"
            echo "  or export FLIGHT_PX4_DIR=<PX4-Autopilot checkout built with make px4_sitl>"
          else
            echo "  PX4/Gazebo: native $FLIGHT_PX4_DIR"
          fi
        '';
      };

      formatter.${system} = pkgs.nixpkgs-fmt;
    };
}
