#mingw tool for testing
from ubuntu:22.04
run apt-get -y update && apt-get -y install g++-mingw-w64-x86-64-posix libboost-dev ninja-build meson&& apt-get -y clean
run apt-get -y install cmake && apt-get clean