# infra/Dockerfile
# -------------------------------------------------------------------
# Use Espressif's official ESP-IDF v5.3 image (includes Python venv & all tools)
# -------------------------------------------------------------------
FROM espressif/idf:release-v5.3

# 1) Install essential OS packages (including tzdata) without prompts
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Etc/UTC
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
       python3-venv python-is-python3 \
       libusb-1.0-0 libusb-1.0-0-dev libftdi1 \
       libsdl2-2.0-0 libsdl2-dev \
       libslirp0 \
       wget curl unzip tzdata \
       sudo pkg-config lsb-release && \
    rm -rf /var/lib/apt/lists/*

# 2) Install Renode (v1.15.3) for Phase 1 onward
RUN wget -q https://github.com/renode/renode/releases/download/v1.15.3/renode-1.15.3.linux-portable.tar.gz && \
    tar -xzf renode-1.15.3.linux-portable.tar.gz -C /opt && \
    ln -s /opt/renode-1.15.3/renode /usr/local/bin/renode && \
    rm renode-1.15.3.linux-portable.tar.gz

# 3) Ensure ESP-IDF is properly installed and virtual environment is set up
RUN bash -lc "cd \$IDF_PATH && ./install.sh esp32"

# 4) Install Python QoL tools inside the ESP-IDF venv (pre-commit, flake8, pytest)
RUN bash -lc "source \$IDF_PATH/export.sh && pip install --upgrade pip pre-commit flake8 pytest"

# 5) Create a non-root user 'dev' and switch into it
RUN useradd -m dev && echo 'dev ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
USER dev
WORKDIR /workspace

# 6) Ensure the ESP-IDF environment is available for the dev user
RUN bash -lc "cd \$IDF_PATH && ./install.sh esp32"
