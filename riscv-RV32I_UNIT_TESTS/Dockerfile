FROM ubuntu:22.04

RUN apt-get update && apt-get -y install sudo

RUN adduser --disabled-password --gecos '' docker
RUN adduser docker sudo
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

USER docker
RUN sudo apt-get update && sudo apt-get -y upgrade
RUN sudo apt-get install -y git

WORKDIR /home/docker

# Install python, pip
RUN sudo apt-get install -y python3
RUN sudo apt-get install -y pip
RUN pip install --upgrade pip

# Install RISCOF
# RUN pip3 install git+https://github.com/riscv/riscof.git

# Install R	ISCV-GNU Toolchain
RUN sudo apt-get install -y autoconf automake autotools-dev curl python3 libmpc-dev \
      libmpfr-dev libgmp-dev gawk build-essential bison flex texinfo gperf libtool \
      patchutils bc zlib1g-dev libexpat-dev

RUN git clone --recursive https://github.com/riscv/riscv-gnu-toolchain
RUN git clone --recursive https://github.com/riscv/riscv-opcodes.git
RUN cd riscv-gnu-toolchain && ./configure --prefix=/opt/riscv --with-arch=rv32gc --with-abi=ilp32d && sudo make

ENV RISCV=/opt/riscv
ENV PATH="$PATH:$RISCV"
ENV PATH="$PATH:$RISCV/bin"
RUN echo $PATH

# RUN sudo apt-get install -y device-tree-compiler
# RUN git clone https://github.com/riscv-software-src/riscv-isa-sim.git
# RUN cd riscv-isa-sim && mkdir build && cd build && ../configure --prefix=$RISCV && make -j8 && sudo make install -j8

RUN sudo apt-get install -y  verilator

# RUN pip3 install git+https://github.com/riscv/riscof.git
ENV PATH="$PATH:/home/docker/.local/bin"

RUN sudo apt-get install -y srecord
