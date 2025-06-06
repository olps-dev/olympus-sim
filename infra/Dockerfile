# Project Olympus - Full Digital Twin Development Environment
# Includes: ESP-IDF, QEMU Xtensa, Renode, ns-3, Gazebo, ROS 2, Python ML stack

FROM ubuntu:22.04

# Prevent interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=UTC

# Create developer user
RUN groupadd --gid 1000 developer && \
    useradd --uid 1000 --gid developer --shell /bin/bash --create-home developer

# Install base system packages
RUN apt-get update && apt-get install -y \
    # Build essentials
    build-essential cmake ninja-build ccache \
    git wget curl unzip \
    # Python and development tools
    python3 python3-pip python3-venv python3-dev \
    # Networking and system tools
    net-tools iproute2 iptables bridge-utils \
    openssh-client telnet netcat-openbsd \
    # Graphics and display
    x11-apps mesa-utils \
    # ESP32 development dependencies
    flex bison gperf \
    libffi-dev libssl-dev dfu-util libusb-1.0-0 \
    # QEMU dependencies
    libglib2.0-dev libpixman-1-dev \
    # Renode dependencies (existing)
    mono-runtime libgtk2.0-0 screen uml-utilities gtk-sharp2 libc6-dev gcc \
    python3-netifaces \
    # ns-3 dependencies
    g++ python3-dev pkg-config sqlite3 \
    libsqlite3-dev libxml2-dev libgtk-3-dev \
    libboost-all-dev \
    # Gazebo dependencies
    lsb-release gnupg2 \
    # ROS 2 dependencies
    software-properties-common \
    # Additional utilities
    vim tmux htop tree jq \
    && rm -rf /var/lib/apt/lists/*

# Install ROS 2 Humble
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    apt-get update && apt-get install -y \
    ros-humble-desktop \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Install Gazebo Garden (latest stable)
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null && \
    apt-get update && apt-get install -y \
    gz-garden \
    && rm -rf /var/lib/apt/lists/*

# Install Python packages for simulation and ML
RUN pip3 install --no-cache-dir \
    # Core simulation packages
    numpy scipy matplotlib seaborn pandas \
    # Network simulation
    ns3 \
    # MQTT and time series
    paho-mqtt influxdb-client \
    # Web frameworks
    flask fastapi uvicorn websockets \
    # Testing frameworks
    pytest pytest-asyncio pytest-mock \
    # Scientific computing
    scikit-learn \
    # Hardware simulation
    pyserial pexpect \
    # Development tools
    black isort mypy pylint \
    # Async and networking
    asyncio-mqtt aiohttp \
    # Configuration
    pyyaml toml \
    # Jupyter for analysis
    jupyter ipython

# Install ESP-IDF v5.3.3 (latest stable)
WORKDIR /opt/esp
RUN git clone --recursive --depth 1 --branch v5.3.3 https://github.com/espressif/esp-idf.git idf && \
    cd idf && \
    ./install.sh esp32 && \
    rm -rf /opt/esp/idf/.git

# Build and install QEMU with Xtensa support
WORKDIR /opt/qemu
RUN git clone --depth 1 --branch stable-8.1 https://gitlab.com/qemu-project/qemu.git && \
    cd qemu && \
    ./configure --target-list=xtensa-softmmu --enable-debug --enable-sanitizers \
                --disable-werror --disable-gtk --disable-sdl --enable-slirp && \
    make -j$(nproc) && \
    make install && \
    rm -rf /opt/qemu/qemu

# Install Renode 1.15.3 (latest stable)
WORKDIR /opt/renode
RUN wget https://github.com/renode/renode/releases/download/v1.15.3/renode-1.15.3.linux-portable.tar.gz && \
    tar -xzf renode-1.15.3.linux-portable.tar.gz && \
    rm renode-1.15.3.linux-portable.tar.gz && \
    ln -s /opt/renode/renode_1.15.3_portable/renode /usr/local/bin/renode

# Build ns-3.42 from source (for stable 802.11s support)
WORKDIR /opt/ns3
RUN wget https://www.nsnam.org/releases/ns-allinone-3.42.tar.bz2 && \
    tar -xjf ns-allinone-3.42.tar.bz2 && \
    cd ns-allinone-3.42/ns-3.42 && \
    ./ns3 configure --build-profile=optimized --enable-examples --enable-tests && \
    ./ns3 build && \
    cp build/lib/lib*.so* /usr/local/lib/ && \
    ldconfig && \
    rm -rf /opt/ns3/ns-allinone-3.42.tar.bz2

# Set up environment
ENV IDF_PATH=/opt/esp/idf
ENV PATH="/opt/esp/idf/tools:${PATH}"
ENV PYTHONPATH="/opt/ns3/ns-allinone-3.42/ns-3.42/src:${PYTHONPATH}"
ENV LD_LIBRARY_PATH="/usr/local/lib:${LD_LIBRARY_PATH}"

# Create workspace structure
WORKDIR /workspace
RUN mkdir -p /workspace/{firmware,sim,tests,results,scripts,infra} && \
    chown -R developer:developer /workspace /home/developer

# Switch to developer user
USER developer

# Source ROS 2 and ESP-IDF in .bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> /home/developer/.bashrc && \
    echo "source /opt/esp/idf/export.sh" >> /home/developer/.bashrc && \
    echo "export ROS_DOMAIN_ID=42" >> /home/developer/.bashrc && \
    echo "cd /workspace" >> /home/developer/.bashrc

# Create Python virtual environment for project
RUN python3 -m venv /home/developer/.venv/olympus && \
    echo "source /home/developer/.venv/olympus/bin/activate" >> /home/developer/.bashrc

WORKDIR /workspace

# Default command
CMD ["/bin/bash", "-l"]
