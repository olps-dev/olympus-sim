FROM ubuntu:22.04

ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    git build-essential cmake ninja-build ccache python3 python3-pip \
    openjdk-11-jdk gdb-multiarch wget curl unzip tzdata \
    software-properties-common ca-certificates sudo pkg-config lsb-release

# ---------- ESP-IDF + QEMU ----------
RUN git clone --branch v5.4.1 --depth 1 https://github.com/espressif/esp-idf.git /opt/esp-idf && \
    /opt/esp-idf/install.sh esp32 && \
    python3 /opt/esp-idf/tools/idf_tools.py install qemu-xtensa
ENV IDF_PATH=/opt/esp-idf
ENV PATH="${IDF_PATH}/tools:${PATH}"

# ---------- Renode ----------
RUN wget -q https://github.com/renode/renode/releases/download/v1.15.3/renode-1.15.3.linux-portable.tar.gz && \
    tar -xzf renode-1.15.3.linux-portable.tar.gz -C /opt && \
    ln -s /opt/renode-1.15.3/renode /usr/local/bin/renode

# ---------- Python QoL ----------
RUN pip3 install pre-commit flake8 pytest
RUN useradd -m dev && echo 'dev ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
USER dev
WORKDIR /workspace
