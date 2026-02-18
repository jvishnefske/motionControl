# CLAUDE.md

## Project

Embedded Rust firmware for the Duet 3 Mini 5+ (ATSAME54P20A) using Embassy async actors.

## Build

```bash
cargo check --workspace          # type-check all crates
cargo test -p gcode-parser --target x86_64-unknown-linux-gnu  # run parser tests on host
cargo clippy --workspace         # lint
cargo fmt --all                  # format
```

## Pull Request Policy

Pull requests should be automatically merged when all CI checks pass. Use `gh pr merge --auto --squash` after creating a PR to enable auto-merge.
