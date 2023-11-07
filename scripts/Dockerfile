ARG version=22.04
FROM buildpack-deps:$version
# this includes requirements for both ceeedling, kas, and bitbake.
run apt-get update && \
    apt-get install -y --no-install-recommends \
    libstdc++-arm-none-eabi-newlib \
    binutils-arm-none-eabi \
    gcc-arm-none-eabi \
    gcovr clang ruby ninja-build cmake python3-pip cppcheck clang-tidy python3 gawk chrpath cpio diffstat locales locales-all && apt-get clean

