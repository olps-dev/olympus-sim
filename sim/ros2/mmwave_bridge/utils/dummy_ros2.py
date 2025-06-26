#!/usr/bin/env python3
"""
Dummy ROS2 classes for testing without ROS2 environment.

These classes mimic the behavior of ROS2 classes for development and testing
when actual ROS2 packages are not available.
"""

import logging

log = logging.getLogger('mmwave_bridge.dummy_ros2')


class Node:
    """Dummy ROS2 Node class when ROS2 is not available."""
    
    def __init__(self, name):
        """Initialize dummy node."""
        self.name = name
        log.info(f"Created dummy node: {name}")
        self._clock = DummyClock()
        
    def get_logger(self):
        """Get a dummy logger that wraps the standard logger."""
        return DummyLogger()
        
    def create_publisher(self, msg_type, topic, qos_profile):
        """Create a dummy publisher."""
        log.info(f"Created dummy publisher for: {topic}")
        return DummyPublisher(topic)
        
    def create_timer(self, period, callback):
        """Create a dummy timer that does nothing."""
        log.info(f"Created dummy timer with period {period}s")
        return DummyTimer()
        
    def destroy_node(self):
        """Clean up resources."""
        log.info(f"Dummy node {self.name} destroyed")
    
    def get_clock(self):
        """Return a dummy clock."""
        return self._clock


class DummyPublisher:
    """Dummy publisher when ROS2 is not available."""
    
    def __init__(self, topic):
        """Initialize with topic name."""
        self.topic = topic
        
    def publish(self, msg):
        """Log message instead of publishing."""
        log.info(f"Dummy publish to {self.topic}: {type(msg).__name__}")


class DummyLogger:
    """Dummy logger when ROS2 is not available."""
    
    def info(self, msg):
        """Log info message."""
        log.info(msg)
        
    def warning(self, msg):
        """Log warning message."""
        log.warning(msg)
        
    def error(self, msg):
        """Log error message."""
        log.error(msg)
        
    def debug(self, msg):
        """Log debug message."""
        log.debug(msg)


class DummyTimer:
    """Dummy timer when ROS2 is not available."""
    
    def cancel(self):
        """Cancel the timer."""
        pass


class DummyClock:
    """Dummy clock when ROS2 is not available."""
    
    def now(self):
        """Return current time."""
        return DummyTime()


class DummyTime:
    """Dummy time when ROS2 is not available."""
    
    def to_msg(self):
        """Convert to message."""
        return {}
