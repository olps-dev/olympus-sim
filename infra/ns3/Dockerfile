# Project Olympus - ns-3 Network Simulation Controller
FROM ubuntu:22.04

# Prevent interactive prompts
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=UTC

# Install dependencies for ns-3
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    ninja-build \
    python3 \
    python3-pip \
    python3-dev \
    git \
    wget \
    curl \
    # ns-3 specific dependencies
    g++ \
    pkg-config \
    sqlite3 \
    libsqlite3-dev \
    libxml2-dev \
    libgtk-3-dev \
    libboost-all-dev \
    # Network tools
    net-tools \
    iproute2 \
    iptables \
    bridge-utils \
    && rm -rf /var/lib/apt/lists/*

# Install Python packages for network simulation
RUN pip3 install --no-cache-dir \
    numpy \
    scipy \
    matplotlib \
    pandas \
    pyyaml \
    asyncio-mqtt \
    aiohttp

# Create working directory
WORKDIR /app

# Copy network simulation files
COPY . /app/

# Set up permissions
RUN chmod +x /app/*.py 2>/dev/null || true

# Default command
CMD ["python3", "mesh_controller.py", "--nodes=5", "--loss_rate=0.05"] 