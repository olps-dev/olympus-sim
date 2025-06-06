#!/usr/bin/env python3
"""
Simple MQTT Visualization for Olympus Simulation
This script subscribes to MQTT topics from the Olympus simulation and displays the data.
"""
import tkinter as tk
import json
import paho.mqtt.client as mqtt
import threading
import time

class SensorDisplay(tk.Frame):
    def __init__(self, master, sensor_id):
        super().__init__(master, bd=2, relief=tk.RAISED, padx=5, pady=5)
        self.sensor_id = sensor_id
        
        # Create labels for sensor data
        self.title_label = tk.Label(self, text=f"Sensor {sensor_id}", font=("Arial", 14, "bold"))
        self.title_label.grid(row=0, column=0, columnspan=2, sticky="w")
        
        # Temperature display
        self.temp_label = tk.Label(self, text="Temperature:")
        self.temp_label.grid(row=1, column=0, sticky="w")
        self.temp_value = tk.Label(self, text="--°C", width=10)
        self.temp_value.grid(row=1, column=1, sticky="e")
        
        # Humidity display
        self.humidity_label = tk.Label(self, text="Humidity:")
        self.humidity_label.grid(row=2, column=0, sticky="w")
        self.humidity_value = tk.Label(self, text="--%", width=10)
        self.humidity_value.grid(row=2, column=1, sticky="e")
        
        # Pressure display
        self.pressure_label = tk.Label(self, text="Pressure:")
        self.pressure_label.grid(row=3, column=0, sticky="w")
        self.pressure_value = tk.Label(self, text="-- hPa", width=10)
        self.pressure_value.grid(row=3, column=1, sticky="e")
        
        # Battery display
        self.battery_label = tk.Label(self, text="Battery:")
        self.battery_label.grid(row=4, column=0, sticky="w")
        self.battery_value = tk.Label(self, text="--V", width=10)
        self.battery_value.grid(row=4, column=1, sticky="e")
        
        # Battery indicator (color changes based on voltage)
        self.battery_indicator = tk.Canvas(self, width=50, height=10, bd=1, relief=tk.SUNKEN)
        self.battery_indicator.grid(row=5, column=0, columnspan=2, sticky="ew", pady=2)
        self.battery_indicator.create_rectangle(0, 0, 50, 10, fill="gray")
        
        # Last update timestamp
        self.last_update = tk.Label(self, text="Last update: Never", font=("Arial", 8))
        self.last_update.grid(row=6, column=0, columnspan=2, sticky="w")
    
    def update_temperature(self, temp_value):
        self.temp_value.config(text=f"{temp_value:.1f}°C")
        # Change color based on temperature
        if temp_value > 30:
            self.temp_value.config(fg="red")
        elif temp_value < 15:
            self.temp_value.config(fg="blue")
        else:
            self.temp_value.config(fg="black")
    
    def update_humidity(self, humidity_value):
        self.humidity_value.config(text=f"{humidity_value:.1f}%")
    
    def update_pressure(self, pressure_value):
        self.pressure_value.config(text=f"{pressure_value:.0f} hPa")
    
    def update_battery(self, voltage):
        self.battery_value.config(text=f"{voltage:.2f}V")
        
        # Update battery indicator
        # Assuming voltage range: 3.0V (empty) to 4.2V (full)
        battery_pct = min(max((voltage - 3.0) / (4.2 - 3.0), 0), 1)
        width = int(50 * battery_pct)
        
        # Color based on level
        if battery_pct > 0.7:
            color = "green"
        elif battery_pct > 0.3:
            color = "yellow"
        else:
            color = "red"
        
        self.battery_indicator.delete("all")
        self.battery_indicator.create_rectangle(0, 0, width, 10, fill=color, outline="")
    
    def update_timestamp(self):
        current_time = time.strftime("%H:%M:%S")
        self.last_update.config(text=f"Last update: {current_time}")


class OlympusVisualizer:
    def __init__(self, broker_host="localhost", broker_port=1883):
        self.root = tk.Tk()
        self.root.title("Olympus Sensor Visualizer")
        self.root.geometry("800x600")
        
        # Create a frame for sensor displays
        self.sensor_frame = tk.Frame(self.root)
        self.sensor_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Create a dictionary to store sensor displays
        self.sensors = {}
        
        # Status bar at the bottom
        self.status_bar = tk.Label(self.root, text="Connecting to MQTT broker...", bd=1, relief=tk.SUNKEN, anchor=tk.W)
        self.status_bar.pack(side=tk.BOTTOM, fill=tk.X)
        
        # Set up MQTT client
        self.mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message
        
        # Connect to broker in a separate thread to avoid blocking the GUI
        self.mqtt_thread = threading.Thread(target=self.connect_mqtt, args=(broker_host, broker_port))
        self.mqtt_thread.daemon = True  # Thread will exit when main program exits
        self.mqtt_thread.start()
        
    def connect_mqtt(self, host, port):
        try:
            self.mqtt_client.connect(host, port, 60)
            self.mqtt_client.loop_start()
            self.root.after(0, lambda: self.status_bar.config(text=f"Connected to MQTT broker at {host}:{port}"))
        except Exception as e:
            self.root.after(0, lambda: self.status_bar.config(text=f"Error connecting to MQTT: {e}"))
    
    def on_connect(self, client, userdata, flags, reason_code, properties):
        if reason_code == 0:
            # Subscribe to all sensor data
            client.subscribe("olympus/sensor_data/#")
            self.status_bar.config(text="Connected to MQTT broker and subscribed to topics")
        else:
            self.status_bar.config(text=f"Failed to connect: {reason_code}")
    
    def on_message(self, client, userdata, msg):
        try:
            # Parse the topic to extract sensor ID and data type
            # Format: olympus/sensor_data/sensor_X/data_type
            topic_parts = msg.topic.split('/')
            if len(topic_parts) >= 4 and topic_parts[0] == "olympus" and topic_parts[1] == "sensor_data":
                sensor_id = topic_parts[2]
                data_type = topic_parts[3]
                
                # Create sensor display if it doesn't exist
                if sensor_id not in self.sensors:
                    self.add_sensor_display(sensor_id)
                
                # Parse payload
                payload = json.loads(msg.payload)
                
                # Update the appropriate sensor display
                self.root.after(0, lambda: self.update_sensor_display(sensor_id, data_type, payload))
        except Exception as e:
            print(f"Error processing message: {e}")
    
    def add_sensor_display(self, sensor_id):
        # Calculate grid position (create a grid of sensors)
        sensor_count = len(self.sensors)
        row = sensor_count // 3
        col = sensor_count % 3
        
        # Create new sensor display
        sensor_display = SensorDisplay(self.sensor_frame, sensor_id)
        sensor_display.grid(row=row, column=col, padx=10, pady=10, sticky="nsew")
        
        # Configure grid weights
        self.sensor_frame.grid_columnconfigure(col, weight=1)
        self.sensor_frame.grid_rowconfigure(row, weight=1)
        
        # Store the sensor display
        self.sensors[sensor_id] = sensor_display
    
    def update_sensor_display(self, sensor_id, data_type, payload):
        sensor = self.sensors[sensor_id]
        
        # Update sensor display based on data type
        if data_type == "bme680":
            if "temperature_c" in payload:
                sensor.update_temperature(payload["temperature_c"])
            if "humidity_rh" in payload:
                sensor.update_humidity(payload["humidity_rh"])
            if "pressure_hpa" in payload:
                sensor.update_pressure(payload["pressure_hpa"])
        elif data_type == "battery":
            if "voltage_v" in payload:
                sensor.update_battery(payload["voltage_v"])
        
        # Update timestamp
        sensor.update_timestamp()
    
    def run(self):
        self.root.mainloop()
        # Clean up when GUI is closed
        self.mqtt_client.loop_stop()


if __name__ == "__main__":
    visualizer = OlympusVisualizer()
    visualizer.run()
