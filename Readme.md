Motion control library
===

skeleton project to send udp packets 

future consideration: implement protocol

build instructions
---

The header may be included in another program, and uses boost asio header only library to communicate with the network on multiple platforms. [Boost header project](https://github.com/jvishnefske/boost-headers) may be added in a subdirectory for easier dependency resolution.

    python3 -m pip install meson ninja
    meson setup build
    cd build
    ninja