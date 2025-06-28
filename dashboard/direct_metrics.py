#!/usr/bin/env python3
"""
Direct metrics reader for dashboard
Reads CSV files and returns real simulation data
"""
import pandas as pd
import numpy as np
from pathlib import Path
import time
import json

def get_real_latency_stats():
    """Get real latency statistics from CSV"""
    try:
        latency_csv = Path("/app/metrics/latency_metrics.csv")
        
        if not latency_csv.exists():
            return {}
            
        df = pd.read_csv(latency_csv)
        
        # Get end-to-end latency data (lamp_toggled events)
        e2e_data = df[
            (df['stage'] == 'lamp_toggled') & 
            df['cumulative_latency_ms'].notna()
        ]
        
        if len(e2e_data) == 0:
            return {}
            
        # Use recent data (last 10 minutes) or all data if no recent
        recent_threshold = time.time() - 600  # 10 minutes
        recent_data = e2e_data[e2e_data['timestamp'] > recent_threshold]
        
        if len(recent_data) > 0:
            latencies = recent_data['cumulative_latency_ms'].values
            data_source = "recent"
        else:
            # Use last 20 events if no recent data
            latencies = e2e_data.tail(20)['cumulative_latency_ms'].values
            data_source = "historical"
            
        if len(latencies) == 0:
            return {}
            
        stats = {
            'mean': float(np.mean(latencies)),
            'p95': float(np.percentile(latencies, 95)),
            'p99': float(np.percentile(latencies, 99)),
            'min': float(np.min(latencies)),
            'max': float(np.max(latencies)),
            'count': int(len(latencies)),
            'data_source': data_source,
            'total_events': int(len(e2e_data))
        }
        
        return stats
        
    except Exception as e:
        print(f"Error reading latency stats: {e}")
        return {}

def get_recent_automation_events():
    """Get recent automation events for display"""
    try:
        latency_csv = Path("/app/metrics/latency_metrics.csv")
        
        if not latency_csv.exists():
            return []
            
        df = pd.read_csv(latency_csv)
        
        # Get recent lamp_toggled events (last 10)
        lamp_events = df[df['stage'] == 'lamp_toggled'].tail(10)
        
        events = []
        for _, row in lamp_events.iterrows():
            events.append({
                'timestamp': row['timestamp'],
                'sensor_id': row['sensor_id'],
                'event_id': row['event_id'],
                'latency_ms': row['cumulative_latency_ms'],
                'action': 'on'  # lamp was toggled
            })
            
        return events
        
    except Exception as e:
        print(f"Error reading automation events: {e}")
        return []

if __name__ == "__main__":
    # Test the functions
    print("Testing direct metrics reader...")
    
    stats = get_real_latency_stats()
    print(f"Latency stats: {stats}")
    
    events = get_recent_automation_events()
    print(f"Recent events: {len(events)}")
    
    if stats:
        print("SUCCESS: Real simulation data available!")
    else:
        print("No latency data found")