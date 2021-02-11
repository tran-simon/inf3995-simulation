FROM ubuntu:18.04


# Install common dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    dpkg \
    git \
    pkg-config \
    python \
    python-dev \
    python-numpy \
    sudo \
    software-properties-common \
    && rm -rf /var/lib/apt/lists/*

# Install ARGoS dependencies
RUN apt-get update && apt-get install -y \
    wget \
    freeglut3-dev \
    qt5-default \
    libxi-dev \
    libxmu-dev \
    libfreeimage-dev \
    libfreeimageplus-dev \
    liblua5.2-dev \
    lua5.2 \
    liblua5.3-dev \
    lua5.3 \
    libboost-filesystem-dev \
    cmake \
    && rm -rf /var/lib/apt/lists/*

# Add dummy argument to force rebuild starting from that point
ARG UPDATE_ARGOS=unknown


WORKDIR /root


# Install Argos from source
RUN git clone https://github.com/MISTLab/argos3.git &&\
    cd argos3 &&\
    git checkout inf3995 &&\
    mkdir build_simulator &&\
    cd build_simulator &&\
    cmake ../src -DCMAKE_BUILD_TYPE=Debug \
     -DARGOS_BUILD_FOR=simulator \
     -DARGOS_THREADSAFE_LOG=ON \
     -DARGOS_DYNAMIC_LOADING=ON &&\
    make -j $(nproc)
RUN touch ./argos3/build_simulator/argos3.1.gz &&\
    touch ./argos3/build_simulator/README.html &&\
    cd ./argos3/build_simulator &&\
    make install
RUN chmod +x ./argos3/build_simulator/argos_post_install.sh &&\
    ./argos3/build_simulator/argos_post_install.sh &&\
    echo "\nsource ./argos3/build_simulator/setup_env.sh\n" >> /.bashrc

#################################
#          YOUR CODE            #
#################################

# Add dummy argument to force rebuild starting from that point
ARG UPDATE_CODE=unknown


COPY ./src ./simulation


#WORKDIR /root

#RUN cd /

# Clone your repository
# If your repository is private, you will need to use ssh keys, look here:
# https://stackoverflow.com/a/23411161/8150481
# For now we clone some argos3 examples
#RUN cd /root &&\
#    git clone https://github.com/MISTLab/argos3-examples.git examples &&\
#    cd examples &&\
#    git checkout inf3995

# Build your code (here examples)
WORKDIR ./simulation
RUN mkdir build && cd build &&\
    cmake -DCMAKE_BUILD_TYPE=Debug .. &&\
    make -j $(nproc)

CMD ["argos3", "-c", "./experiments/crazyflie_sensing.argos"]
