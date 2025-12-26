# Motion Control

A cross-platform C++ UDP networking library for motion control applications. Built on Boost.Asio, it provides asynchronous packet transmission for real-time motion data streaming to simulators and control systems.

## Quick Start

```bash
make build      # Configure and build with CMake
make test       # Run unit tests
make coverage   # Generate coverage report
```

## Features

- Asynchronous UDP client with configurable server endpoint
- Packet handler callbacks for incoming data
- Vector data transmission for motion coordinates
- Cross-platform support (Linux, Windows via MinGW)

## Dependencies

- C++17 compiler
- CMake 3.25+
- Boost (asio headers)
- GoogleTest (fetched automatically)
- Google Benchmark (fetched automatically)

## Build Options

### CMake (Recommended)

```bash
mkdir build && cd build
cmake ..
cmake --build .
```

### Meson (Alternative)

```bash
python3 -m pip install meson ninja
meson setup build
cd build
ninja
```

## License

See LICENSE file for details.
