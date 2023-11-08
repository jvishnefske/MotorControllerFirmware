ARG version=20.04
ARG ARM_URL="https://developer.arm.com/-/media/Files/downloads/gnu/13.2.rel1/binrel/arm-gnu-toolchain-13.2.rel1-x86_64-arm-none-eabi.tar.xz"
FROM buildpack-deps:$version
ARG ARM_URL
# this includes requirements for both ceeedling, kas, and bitbake.
run apt-get update && \
    apt-get install -y --no-install-recommends \
    gcovr clang ruby python3-pip \
    libncurses5 libncursesw5\
    cppcheck clang-tidy python3 gawk chrpath cpio diffstat locales locales-all && \
    apt-get clean
run wget $ARM_URL --progress=dot:giga -O gcc-arm-none-eabi.tar.xz &&\
    mkdir gcc-arm-none-eabi && tar xJfv gcc-arm-none-eabi.tar.xz -C gcc-arm-none-eabi --strip-components 1 &&\
    rm gcc-arm-none-eabi.tar.xz
run python3 -m pip install cmake ninja
ENV PATH="/gcc-arm-none-eabi/bin:${PATH}:/gcc-arm-none-eabi/arm-none-eabi/bin/"
