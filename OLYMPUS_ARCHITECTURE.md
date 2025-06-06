# Project Olympus - Complete Digital Twin Architecture
## Smart Home Simulation Platform with Hardware-Accurate Fidelity

---

## 🎯 **Executive Summary**

Project Olympus is a comprehensive digital twin platform for validating smart home IoT systems before hardware deployment. The system provides instruction-level ESP32 simulation, realistic 802.11s mesh networking, physical world modeling, and complete backend stack integration—all running on a single laptop for comprehensive development and CI/CD validation.

**Key Achievement**: Eliminates hardware surprises by running exact firmware in a deterministic sandbox that validates bus-level bugs, timing constraints, battery life projections, and end-to-end latency before any board arrives.

---

## 🏗️ **System Architecture Overview**

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                        PROJECT OLYMPUS DIGITAL TWIN                        │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐           │
│  │   PHYSICAL      │  │    NETWORK      │  │    BACKEND      │           │
│  │   SIMULATION    │  │   SIMULATION    │  │     STACK       │           │
│  │                 │  │                 │  │                 │           │
│  │ ┌─────────────┐ │  │ ┌─────────────┐ │  │ ┌─────────────┐ │           │
│  │ │   Gazebo    │ │  │ │    ns-3     │ │  │ │ Mosquitto   │ │           │
│  │ │ Apartment   │ │  │ │ 802.11s     │ │  │ │   (MQTT)    │ │           │
│  │ │   World     │ │  │ │    Mesh     │ │  │ │             │ │           │
│  │ │             │ │  │ │             │ │  │ └─────────────┘ │           │
│  │ │ • Actors    │ │  │ │ • TAP       │ │  │ ┌─────────────┐ │           │
│  │ │ • Sensors   │ │  │ │   Bridges   │ │  │ │  InfluxDB   │ │           │
│  │ │ • Rooms     │ │  │ │ • Packet    │ │  │ │ Time Series │ │           │
│  │ └─────────────┘ │  │ │   Loss      │ │  │ │             │ │           │
│  └─────────────────┘  │ │ • Latency   │ │  │ └─────────────┘ │           │
│                       │ │   Models    │ │  │ ┌─────────────┐ │           │
│                       │ └─────────────┘ │  │ │  Node-RED   │ │           │
│                       └─────────────────┘  │ │ Rule Logic  │ │           │
│                                            │ │             │ │           │
│                                            │ └─────────────┘ │           │
│                                            │ ┌─────────────┐ │           │
│                                            │ │    IRIS     │ │           │
│                                            │ │ Dashboard   │ │           │
│                                            │ │             │ │           │
│                                            │ └─────────────┘ │           │
│                                            └─────────────────┘           │
│                                                                             │
├─────────────────────────────────────────────────────────────────────────────┤
│                           HARDWARE SIMULATION LAYER                        │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│ ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐              │
│ │   ESP32 NODE 0  │  │   ESP32 NODE 1  │  │   ESP32 NODE N  │              │
│ │                 │  │                 │  │                 │              │
│ │ ┌─────────────┐ │  │ ┌─────────────┐ │  │ ┌─────────────┐ │              │
│ │ │ QEMU Xtensa │ │  │ │ QEMU Xtensa │ │  │ │ QEMU Xtensa │ │              │
│ │ │   Engine    │ │  │ │   Engine    │ │  │ │   Engine    │ │              │
│ │ │             │ │  │ │             │ │  │ │             │ │              │
│ │ │ • Real ESP32│ │  │ │ • Real ESP32│ │  │ │ • Real ESP32│ │              │
│ │ │   Firmware  │ │  │ │   Firmware  │ │  │ │   Firmware  │ │              │
│ │ │ • Dual Core │ │  │ │ • Dual Core │ │  │ │ • Dual Core │ │              │
│ │ │ • FreeRTOS  │ │  │ │ • FreeRTOS  │ │  │ │ • FreeRTOS  │ │              │
│ │ └─────────────┘ │  │ └─────────────┘ │  │ └─────────────┘ │              │
│ │                 │  │                 │  │                 │              │
│ │ ┌─────────────┐ │  │ ┌─────────────┐ │  │ ┌─────────────┐ │              │
│ │ │ Peripheral  │ │  │ │ Peripheral  │ │  │ │ Peripheral  │ │              │
│ │ │   Bridge    │ │  │ │   Bridge    │ │  │ │   Bridge    │ │              │
│ │ │             │ │  │ │             │ │  │ │             │ │              │
│ │ │ • BME680    │ │  │ │ • BME680    │ │  │ │ • BME680    │ │              │
│ │ │ • SSD1306   │ │  │ │ • SSD1306   │ │  │ │ • SSD1306   │ │              │
│ │ │ • Battery   │ │  │ │ • Battery   │ │  │ │ • Battery   │ │              │
│ │ │ • I2C/SPI   │ │  │ │ • I2C/SPI   │ │  │ │ • I2C/SPI   │ │              │
│ │ │   Timing    │ │  │ │   Timing    │ │  │ │   Timing    │ │              │
│ │ └─────────────┘ │  │ └─────────────┘ │  │ └─────────────┘ │              │
│ └─────────────────┘  └─────────────────┘  └─────────────────┘              │
│                                                                             │
├─────────────────────────────────────────────────────────────────────────────┤
│                      ORCHESTRATION & MONITORING LAYER                      │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│ ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐              │
│ │   ROS 2 TIME    │  │   KPI MONITOR   │  │     CI/CD       │              │
│ │     SYNC        │  │                 │  │  INTEGRATION    │              │
│ │                 │  │ • Latency P95   │  │                 │              │
│ │ • Gazebo ↔      │  │ • Battery Life  │  │ • Automated     │              │
│ │   ns-3 ↔ QEMU   │  │ • Packet Loss   │  │   Testing       │              │
│ │ • World Time    │  │ • Real-time     │  │ • KPI           │              │
│ │   Coordination  │  │   Factor        │  │   Validation    │              │
│ │ • Actor         │  │ • Detection     │  │ • Performance   │              │
│ │   Triggering    │  │   Accuracy      │  │   Benchmarks    │              │
│ └─────────────────┘  └─────────────────┘  └─────────────────┘              │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## 🔧 **Technical Components Deep Dive**

### **1. Hardware Simulation Layer**

#### **QEMU ESP32 Xtensa Engine**
- **Purpose**: Instruction-level accurate ESP32 execution
- **Technology**: QEMU 8.1 with Xtensa core support
- **Capabilities**:
  - Dual-core ESP32-WROOM-32 simulation
  - Real ESP-IDF firmware execution
  - Hardware-accurate register access
  - GDB debugging support
  - TAP network interface integration

#### **Peripheral Bridge System**
- **Purpose**: Hardware-accurate sensor simulation
- **Technology**: Python asyncio with realistic timing models
- **Sensors Supported**:
  - **BME680**: Environmental sensing (temp, humidity, IAQ)
  - **SSD1306**: OLED display with text rendering
  - **Battery ADC**: Li-ion discharge curve modeling
  - **mmWave Radar**: Human presence detection (future)

#### **Bus Protocol Simulation**
- **I2C**: 100kHz timing, NACK simulation, multi-device support
- **SPI**: 10MHz with CS timing, transaction queuing
- **ADC**: 12-bit resolution, noise modeling, calibration
- **Error Injection**: Realistic failure modes for robustness testing

### **2. Network Simulation Layer**

#### **ns-3 802.11s Mesh Network**
- **Purpose**: Realistic wireless mesh networking
- **Technology**: ns-3 3.42 with stable 802.11s support
- **Features**:
  - Indoor path loss modeling
  - Wall attenuation simulation
  - Packet loss and retransmission
  - RSSI-based link quality
  - TAP bridge integration for ESP32 connectivity

#### **Network Performance Modeling**
```python
# Realistic apartment mesh characteristics
Indoor Path Loss: -30dB - 20*log10(distance) - wall_penalty
Packet Loss Rate: RSSI-dependent (1% @ -50dBm, 50% @ -90dBm)
Mesh Latency: 1-15ms base + queuing delays
Throughput: 54Mbps * (1 - packet_loss_rate)
```

### **3. Physical World Simulation**

#### **Gazebo Apartment World**
- **Purpose**: Realistic smart home environment
- **Technology**: Gazebo Garden with ROS 2 integration
- **Environment**:
  - Multi-room apartment layout (living room, bedrooms, kitchen)
  - Realistic furniture and wall placements
  - Dynamic lighting simulation
  - Physics-based collision detection

#### **Actor System**
- **Human Actors**: Scripted movement patterns
- **Detection Triggers**: Proximity-based sensor activation
- **Behavioral Modeling**: Realistic occupancy patterns
- **Integration**: ROS 2 messaging for sensor triggers

### **4. Backend Stack**

#### **MQTT Broker (Mosquitto)**
- **Purpose**: IoT message routing and persistence
- **Configuration**: Optimized for sensor data throughput
- **Features**: WebSocket support, message persistence, QoS handling

#### **Time Series Database (InfluxDB)**
- **Purpose**: Sensor data storage and analysis
- **Capabilities**: High-throughput ingestion, real-time queries
- **Integration**: Automated KPI collection and alerting

#### **Rule Engine (Node-RED)**
- **Purpose**: Smart home automation logic
- **Features**: Visual flow programming, MQTT integration
- **Use Cases**: Sensor fusion, automated responses, alerts

#### **Dashboard (IRIS)**
- **Purpose**: Real-time system monitoring and control
- **Technology**: Modern web interface with live updates
- **Features**: Sensor visualization, network topology, KPI dashboards

### **5. Orchestration & Monitoring**

#### **ROS 2 Time Synchronization Bridge**
- **Purpose**: Coordinate simulation time across all components
- **Technology**: ROS 2 Humble with custom synchronization
- **Responsibilities**:
  - Gazebo world time → ns-3 simulation time
  - Actor position → sensor trigger coordination
  - MQTT message timestamping
  - KPI data collection and validation

#### **KPI Monitoring System**
```python
# Validated KPI Thresholds
End-to-End Latency: <300ms (P95)
Battery Life Projection: ≥9 days (216 hours)
Mesh Packet Delivery: >95%
Real-Time Factor: >0.8
Detection Accuracy: >95%
```

---

## 🚀 **Usage Patterns**

### **For Investor Demonstrations**
```bash
make full_sim
# Visit http://localhost:8080 for IRIS dashboard
# Watch virtual apartment with sensor triggers
# Observe <300ms end-to-end response times
```

### **For CI/CD Integration**
```bash
make ci_full_sim
# Runs 5-minute simulation with KPI validation
# Exits with failure if any threshold is violated
# Generates performance benchmarks
```

### **For Development Iteration**
```bash
make demo && make hardware_test
# Quick ESP32 simulation + hardware validation
# Perfect for rapid development cycles
```

### **For Hardware Validation**
```bash
make hw_bridge && make validate_protocols
# Hardware-accurate I2C/SPI timing
# Bus protocol compliance testing
# Power consumption validation
```

---

## 📊 **Key Performance Indicators (KPIs)**

### **Latency Requirements**
- **Sensor-to-MQTT**: <50ms (hardware processing + mesh transmission)
- **MQTT-to-Dashboard**: <20ms (backend processing)
- **Total Pipeline**: <300ms (P95 requirement for smart home responsiveness)

### **Power Consumption Modeling**
```python
# ESP32 Power States (realistic measurements)
Active WiFi Transmission: 160mW
WiFi Connected (idle): 20mW  
Deep Sleep: 0.01mW

# Duty Cycle Optimization
Active Time: 10% (sensor readings, mesh communication)
Idle Time: 30% (WiFi connected, waiting)
Sleep Time: 60% (deep sleep between readings)

# Battery Life Calculation
Average Power: 16.01mW
18650 Battery: 11,100mWh capacity
Projected Life: 693 hours (28.9 days) ✅
```

### **Network Performance Requirements**
- **Packet Delivery Ratio**: >95% (ensures reliable sensor data)
- **Mesh Latency**: <100ms (rapid inter-node communication)
- **Network Throughput**: >20Mbps effective (supports video streaming)

### **System Reliability Metrics**
- **Sensor Error Rate**: <0.1% (hardware fault tolerance)
- **Detection Accuracy**: >95% (mmWave radar performance)
- **Real-Time Factor**: >0.8 (simulation keeps up with real-time)

---

## 🛠️ **Development Workflow**

### **1. Firmware Development**
```bash
# Build ESP32 firmware
make build

# Test with hardware-accurate simulation
make hw_bridge

# Validate I2C/SPI protocols
make validate_protocols
```

### **2. Network Testing**
```bash
# Test mesh network performance
make test_network

# Validate packet delivery ratio
make validate_mesh

# Monitor network stats
make monitor_kpis
```

### **3. Integration Testing**
```bash
# Full end-to-end testing
make test_integration

# KPI validation
make validate_latency
make validate_battery

# Complete simulation test
make test_full_sim
```

### **4. Production Validation**
```bash
# CI/CD pipeline testing
make ci_test

# Performance benchmarking
make benchmark

# Hardware regression testing
make regression
```

---

## 🎯 **Competitive Advantages**

### **1. Hardware Surprises Eliminated**
- Real firmware execution catches bus-level bugs in simulation
- Timing constraints validated before hardware arrives
- Power consumption projected with realistic duty cycling

### **2. Investor Demo Ready**
- Virtual apartment with human actors triggers realistic sensor responses
- <300ms end-to-end latency demonstrated visually
- Complete system behavior without requiring physical hardware

### **3. CI/CD Pipeline Integration**
- Every code change triggers full virtual house simulation
- Automated KPI validation prevents performance regressions
- Hardware-level regression testing catches integration bugs

### **4. Development Velocity**
- Parallel development of firmware, networking, and backend
- No hardware dependencies for integration testing
- Rapid iteration with deterministic simulation environment

---

## 📋 **Implementation Status**

### **✅ Completed Components**
- **QEMU ESP32 Xtensa Simulation**: Full instruction-level accuracy
- **Hardware-Accurate Peripheral Bridge**: I2C/SPI timing, error injection
- **ns-3 802.11s Mesh Controller**: Realistic apartment topology
- **Gazebo Apartment World**: Multi-room environment with actors
- **Complete Backend Stack**: MQTT, InfluxDB, Node-RED, IRIS
- **ROS 2 Time Synchronization**: Cross-component coordination
- **Comprehensive Testing Framework**: KPI validation, CI integration
- **Production Build System**: One-command simulation startup

### **🔄 Areas for Future Enhancement**
- **mmWave Radar Simulation**: Detailed point cloud generation
- **Multi-Building Mesh**: Scaling beyond single apartment
- **Cloud Integration**: Hybrid local/cloud processing
- **Machine Learning Integration**: Occupancy prediction, anomaly detection

---

## 🎉 **Bottom Line Achievement**

**Project Olympus successfully transforms the risky smart home development process into a deterministic, hardware-accurate simulation environment.** 

The system validates:
- ✅ **Real ESP32 firmware execution** with instruction-level accuracy
- ✅ **Hardware-accurate I2C/SPI protocols** with realistic timing
- ✅ **802.11s mesh networking** with apartment-scale propagation
- ✅ **Complete backend integration** with MQTT, time series, and dashboards
- ✅ **End-to-end latency <300ms** with comprehensive KPI monitoring
- ✅ **Battery life ≥9 days** with realistic power modeling
- ✅ **CI/CD integration** with automated regression testing

**The result**: Hardware bring-up becomes a validation step rather than a debug session, accelerating development velocity and eliminating costly hardware surprises. 