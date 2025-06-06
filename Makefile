.PHONY: sim build shell clean sim_phase1 sim_with_bridge test demo hardware_test hw_bridge full_sim startup_stack shutdown_stack ros2_sim ros2_gazebo_sim ros2_setup

DOCKER_COMPOSE = docker compose -f infra/docker-compose.yml

# =============================================================================
# DEVELOPMENT TARGETS
# =============================================================================

build:
	$(DOCKER_COMPOSE) run --rm \
	  dev \
	  bash -lc 'cd /workspace/firmware/hello_world && . $$IDF_PATH/export.sh && idf.py build'

shell:
	$(DOCKER_COMPOSE) run --rm dev bash

clean:
	$(DOCKER_COMPOSE) run --rm \
	  dev \
	  bash -lc 'cd /workspace/firmware/hello_world && . $$IDF_PATH/export.sh && idf.py clean'

# =============================================================================
# FULL OLYMPUS SIMULATION TARGETS
# =============================================================================

full_sim: startup_stack
	@echo "Starting Project Olympus Digital Twin Simulation"
	@echo "================================================="
	@echo "Backend services started. Starting physics simulation..."
	@echo ""
	@echo "Access Points:"
	@echo "   * IRIS Dashboard: http://localhost:8080"
	@echo "   * Node-RED: http://localhost:1880"
	@echo "   * InfluxDB: http://localhost:8086"
	@echo "   * UART Output: telnet localhost 3333"
	@echo ""
	@echo "The physics simulation will run in foreground."
	@echo "Press Ctrl+C to stop."
	@echo "================================================="
	@echo
	docker run --rm --network infra_olympus_backend -p 3333:3333 -v ${PWD}:/workspace python:3.11-slim bash -c "pip install paho-mqtt numpy && python3 /workspace/scripts/physics_simulation.py"

startup_stack:
	@echo "Starting Olympus Backend Stack..."
	$(DOCKER_COMPOSE) up -d mqtt influxdb nodered iris metrics
	@echo "Waiting for services to initialize..."
	@sleep 15
	@echo "Backend stack ready"
	@echo "Check running services with: docker compose -f infra/docker-compose.yml ps"

shutdown_stack:
	@echo "Shutting down Olympus simulation stack..."
	$(DOCKER_COMPOSE) down -v
	@docker system prune -f
	@echo "Cleanup complete"

start_esp32_nodes:
	@echo "Starting ESP32 QEMU nodes..."
	@echo "WARNING: This requires built firmware at /workspace/firmware/build/olympus.elf"
	@echo "If firmware doesn't exist, containers will fail to start"
	@echo "Use 'make demo' instead for working sensor simulation"
	# Start 5 ESP32 nodes with staggered initialization
	$(DOCKER_COMPOSE) run -d --name olympus-esp32-0 dev python3 /workspace/sim/qemu/esp32_runner.py /workspace/firmware/build/olympus.elf --node-id 0 --uart-port 3333 &
	@sleep 2
	$(DOCKER_COMPOSE) run -d --name olympus-esp32-1 dev python3 /workspace/sim/qemu/esp32_runner.py /workspace/firmware/build/olympus.elf --node-id 1 --uart-port 3334 &
	@sleep 2
	$(DOCKER_COMPOSE) run -d --name olympus-esp32-2 dev python3 /workspace/sim/qemu/esp32_runner.py /workspace/firmware/build/olympus.elf --node-id 2 --uart-port 3335 &
	@sleep 2
	$(DOCKER_COMPOSE) run -d --name olympus-esp32-3 dev python3 /workspace/sim/qemu/esp32_runner.py /workspace/firmware/build/olympus.elf --node-id 3 --uart-port 3336 &
	@sleep 2
	$(DOCKER_COMPOSE) run -d --name olympus-esp32-4 dev python3 /workspace/sim/qemu/esp32_runner.py /workspace/firmware/build/olympus.elf --node-id 4 --uart-port 3337 &
	@echo "ESP32 nodes starting (if firmware exists)..."

test_full_sim:
	@echo "Running Basic Simulation Tests"
	@echo "This validates the available components:"
	@echo "  * Backend services connectivity"
	@echo "  * MQTT message flow"
	@echo "  * Dashboard accessibility"
	$(DOCKER_COMPOSE) run --rm dev bash -c "cd /workspace && python3 -m pytest tests/test_full_simulation.py::test_simulation_startup tests/test_full_simulation.py::test_dashboard_accessibility -v --tb=short"

# =============================================================================
# BASIC SIMULATION TARGETS (Legacy)
# =============================================================================

sim:
	$(DOCKER_COMPOSE) run --rm \
	  dev \
	  bash -lc 'cd /workspace/firmware/hello_world && . $$IDF_PATH/export.sh && idf.py qemu'

sim_phase1:
	$(DOCKER_COMPOSE) run --rm dev bash -c "cd /workspace/firmware && . \$$IDF_PATH/export.sh && idf.py build && renode -e 'i @sim/renode/run.resc'"

sim_with_bridge:
	@echo "Starting simulation with sensor bridge..."
	@echo "This will start Renode in the background and run the sensor bridge"
	$(DOCKER_COMPOSE) run --rm -d --name olympus-sim dev bash -c "cd /workspace/firmware && . \$$IDF_PATH/export.sh && idf.py build && renode -e 'i @sim/renode/run.resc'" && \
	sleep 5 && \
	$(DOCKER_COMPOSE) run --rm dev python3 /workspace/sim/python/sensor_bridge.py || \
	(docker stop olympus-sim 2>/dev/null; docker rm olympus-sim 2>/dev/null; exit 1)

hw_bridge:
	@echo "🔧 Starting Hardware-Accurate Sensor Bridge"
	@echo "Real I2C/SPI/ADC timing, register-level behavior, error injection"
	@echo "This provides hardware-level fidelity for proper digital twin testing"
	$(DOCKER_COMPOSE) run --rm -d --name olympus-hw-sim dev bash -c "cd /workspace/firmware && . \$$IDF_PATH/export.sh && idf.py build && renode -e 'i @sim/renode/run.resc'" && \
	sleep 8 && \
	$(DOCKER_COMPOSE) run --rm -p 3333:3333 dev python3 /workspace/sim/python/hardware_sensor_bridge.py || \
	(docker stop olympus-hw-sim 2>/dev/null; docker rm olympus-hw-sim 2>/dev/null; exit 1)

demo:
	@echo "Starting Project Olympus Physics-Based Digital Twin Simulation"
	@echo "Real environmental physics simulation with correlated sensor responses"
	@echo "Connect to TCP port 3333 to see UART output (telnet localhost 3333)"
	@echo "View real-time data at: http://localhost:8080"
	@echo "Physics simulation includes:"
	@echo "  * Realistic day/night cycles (1 hour = 10 seconds)"
	@echo "  * Human occupancy patterns"
	@echo "  * Physics-based sensor correlations"
	@echo "  * Battery discharge simulation"
	@echo "  * Motion detection based on occupancy"
	docker run --rm --network infra_olympus_backend -p 3333:3333 -v ${PWD}:/workspace python:3.11-slim bash -c "pip install paho-mqtt numpy && python3 /workspace/scripts/physics_simulation.py"

# =============================================================================
# ROS2 AND GAZEBO INTEGRATION TARGETS
# =============================================================================

ros2_setup:
	@echo "Setting up ROS2 environment..."
	@chmod +x scripts/run_integrated_simulation.sh

ros2_sim: ros2_setup
	@echo "Starting Olympus simulation with ROS2 integration..."
	@echo "This runs the simulation with ROS2 but without Gazebo"
	python3 sim/python/core/main_simulation_ros2.py --ros2

ros2_gazebo_sim: ros2_setup
	@echo "Starting full Olympus simulation with ROS2 and Gazebo integration..."
	@echo "This launches ROS2, Gazebo, and the Olympus simulation"
	./scripts/run_integrated_simulation.sh

# =============================================================================
# TESTING TARGETS
# =============================================================================

test:
	$(DOCKER_COMPOSE) run --rm dev bash -c "cd /workspace && python3 -m pytest tests/test_peripherals.py -v"

hardware_test:
	@echo "🔬 Running Hardware-Level Integration Tests"
	@echo "Testing actual bus transactions, timing, error conditions, and register-level behavior"
	@echo "This validates hardware-accurate simulation and catches real hardware bugs"
	$(DOCKER_COMPOSE) run --rm dev bash -c "cd /workspace && python3 -m pytest tests/test_hardware_level.py -v --tb=short"

test_all:
	@echo "🧪 Running Complete Test Suite"
	@echo "Surface-level tests + Hardware-level validation + Full simulation tests"
	$(DOCKER_COMPOSE) run --rm dev bash -c "cd /workspace && python3 -m pytest tests/ -v --tb=short"

test_network:
	@echo " Testing ns-3 Mesh Network Simulation"
	$(DOCKER_COMPOSE) run --rm dev bash -c "cd /workspace && python3 -m pytest tests/test_mesh_network.py -v"

test_gazebo:
	@echo " Testing Gazebo World Simulation"
	$(DOCKER_COMPOSE) run --rm dev bash -c "cd /workspace && python3 -m pytest tests/./scripts/launch/launch_ros_gazebo.sh_world.py -v"

test_integration:
	@echo " Running Full Integration Tests"
	@echo "End-to-end validation of actor detection → sensor triggers → MQTT → dashboard"
	$(DOCKER_COMPOSE) run --rm dev bash -c "cd /workspace && python3 -m pytest tests/test_integration.py -v --tb=short"

# =============================================================================
# VALIDATION TARGETS
# =============================================================================

validate_power:
	@echo " Power Consumption and Timing Validation"
	@echo "Validates power models, timing constraints, and battery life predictions"
	$(DOCKER_COMPOSE) run --rm dev bash -c "cd /workspace && python3 -m pytest tests/test_hardware_level.py::test_power_consumption_timing -v"

validate_protocols:
	@echo " Bus Protocol Compliance Testing"
	@echo "I2C/SPI timing validation, error injection, NACK handling"
	$(DOCKER_COMPOSE) run --rm dev bash -c "cd /workspace && python3 -m pytest tests/test_hardware_level.py::test_i2c_bus_protocol_compliance tests/test_hardware_level.py::test_spi_bus_protocol_compliance -v"

validate_latency:
	@echo "  End-to-End Latency Validation"
	@echo "Validates <300ms sensor-to-dashboard pipeline requirement"
	$(DOCKER_COMPOSE) run --rm dev bash -c "cd /workspace && python3 -m pytest tests/test_full_simulation.py::test_end_to_end_latency -v"

validate_battery:
	@echo " Battery Life Validation"
	@echo "Validates ≥9 day battery life requirement with realistic power models"
	$(DOCKER_COMPOSE) run --rm dev bash -c "cd /workspace && python3 -m pytest tests/test_full_simulation.py::test_battery_life_projection -v"

validate_mesh:
	@echo " Mesh Network Validation"
	@echo "Validates >95% packet delivery ratio and <100ms mesh latency"
	$(DOCKER_COMPOSE) run --rm dev bash -c "cd /workspace && python3 -m pytest tests/test_mesh_network.py::test_packet_delivery_ratio -v"

# =============================================================================
# CI/CD TARGETS
# =============================================================================

ci_test:
	@echo " CI/CD Pipeline Test Suite"
	@echo "Automated testing for continuous integration"
	$(DOCKER_COMPOSE) run --rm dev bash -c "cd /workspace && python3 -m pytest tests/ -v --tb=line --maxfail=3 --timeout=600"

regression:
	@echo " Regression Testing Suite"
	@echo "Complete hardware-level regression tests for CI/CD pipeline"
	@echo "Validates that code changes don't break hardware behavior"
	$(DOCKER_COMPOSE) run --rm dev bash -c "cd /workspace && python3 -m pytest tests/test_hardware_level.py -v --tb=line --maxfail=1"

ci_full_sim:
	@echo " CI Full Simulation Test"
	@echo "Runs complete simulation for 5 minutes and validates all KPIs"
	@echo "Designed for automated CI/CD pipeline validation"
	$(DOCKER_COMPOSE) run --rm --env CI_MODE=true dev bash -c "\
		cd /workspace && \
		timeout 300 make full_sim & \
		sleep 60 && \
		python3 -m pytest tests/test_full_simulation.py::test_ci_kpi_validation -v --tb=short && \
		pkill -f 'full_sim' || true"

benchmark:
	@echo " Performance Benchmarking"
	@echo "Measures simulation performance and generates benchmark report"
	$(DOCKER_COMPOSE) run --rm dev bash -c "cd /workspace && python3 ./scripts/launch/launch_integrated_simulation.shpy --output results/benchmark_$(shell date +%Y%m%d_%H%M%S).json"

# =============================================================================
# DEBUGGING AND ANALYSIS TARGETS
# =============================================================================

debug_hardware:
	@echo " Hardware Debug Mode"
	@echo "🐛 Hardware Debug Mode"
	@echo "Interactive session with hardware monitoring enabled"
	$(DOCKER_COMPOSE) run --rm -it dev bash -c "cd /workspace && python3 -c 'from tests.test_hardware_level import hardware_monitor; import IPython; IPython.embed()'"

debug_full_sim:
	@echo "🔍 Full Simulation Debug Mode"
	@echo "Interactive debugging session with live simulation monitoring"
	$(DOCKER_COMPOSE) run --rm -it dev bash -c "cd /workspace && python3 scripts/debug_full_simulation.py"

monitor_kpis:
	@echo "📈 Live KPI Monitoring"
	@echo "Real-time monitoring of simulation KPIs"
	$(DOCKER_COMPOSE) run --rm dev python3 scripts/monitor_kpis.py

logs:
	@echo "📋 Showing simulation logs..."
	$(DOCKER_COMPOSE) logs -f

# =============================================================================
# CLEANUP TARGETS
# =============================================================================

stop_sim:
	@echo "🛑 Stopping all simulation processes..."
	@docker stop olympus-sim 2>/dev/null || true
	@docker rm olympus-sim 2>/dev/null || true
	@docker stop olympus-hw-sim 2>/dev/null || true
	@docker rm olympus-hw-sim 2>/dev/null || true
	@for i in 0 1 2 3 4; do \
		docker stop olympus-esp32-$$i 2>/dev/null || true; \
		docker rm olympus-esp32-$$i 2>/dev/null || true; \
	done
	@echo "✅ All simulation processes stopped"

clean_all: stop_sim shutdown_stack
	@echo "🧹 Deep cleaning all simulation artifacts..."
	@docker system prune -af --volumes
	@docker network prune -f
	@echo "✅ Deep cleanup complete"

# =============================================================================
# HELP AND DOCUMENTATION
# =============================================================================

help:
	@echo "Project Olympus - Digital Twin Build System"
	@echo "======================================================================"
	@echo ""
	@echo "MAIN TARGETS:"
	@echo "  full_sim       - Start complete simulation (backend + physics demo)"
	@echo "  startup_stack  - Start backend services only (MQTT, InfluxDB, IRIS)"
	@echo "  demo           - Physics-based sensor simulation with MQTT"
	@echo ""
	@echo "ACCESS POINTS:"
	@echo "  IRIS Dashboard: http://localhost:8080"
	@echo "  Node-RED:       http://localhost:1880"
	@echo "  InfluxDB:       http://localhost:8086"
	@echo "  UART Output:    telnet localhost 3333"
	@echo ""
	@echo "DEVELOPMENT:"
	@echo "  dev_shell      - Enter development container"
	@echo "  build_stack    - Build all Docker images"
	@echo "  stop_sim       - Stop all simulation services"
	@echo "  clean_sim      - Clean up simulation data"
	@echo ""
	@echo "SYSTEM REQUIREMENTS:"
	@echo "  * Docker with compose support"
	@echo "  * 4GB+ RAM available"
	@echo "  * Ports 1883, 8080, 8086, 1880, 3333 available"

startup_guide:
	@echo ""
	@echo "PROJECT OLYMPUS - QUICK START GUIDE"
	@echo "===================================="
	@echo ""
	@echo "STEP 1: Start Backend Services"
	@echo "  make startup_stack"
	@echo ""
	@echo "STEP 2: Run Physics Simulation"
	@echo "  make demo"
	@echo ""
	@echo "STEP 3: Access Interfaces"
	@echo "  IRIS Dashboard: http://localhost:8080"
	@echo "  UART Output:    telnet localhost 3333"
	@echo ""
	@echo "STEP 4: Stop Everything"
	@echo "  make stop_sim"
	@echo ""
	@echo "The simulation includes:"
	@echo "  * 5 ESP32 nodes with physics-accurate sensors"
	@echo "  * Real environmental modeling (day/night, occupancy)"
	@echo "  * Realistic battery discharge curves"
	@echo "  * Motion detection based on human presence"
	@echo "  * Real-time KPI monitoring and validation"
	@echo ""
