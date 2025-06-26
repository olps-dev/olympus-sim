#!/usr/bin/env python3
"""
Logging utilities for the mmWave ROS2 bridge.

Provides standardized logging setup across the package.
"""

import os
import logging
import sys


def setup_logger(name: str, level=logging.DEBUG):
    """
    Set up a logger with console and file output.
    
    Args:
        name: Logger name
        level: Logging level
        
    Returns:
        Configured logger instance
    """
    logger = logging.getLogger(name)
    logger.setLevel(level)
    
    # Only add handlers if they don't exist already
    if not logger.handlers:
        # Console handler
        console_handler = logging.StreamHandler()
        console_formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        console_handler.setFormatter(console_formatter)
        logger.addHandler(console_handler)
        
        # File handler - log to a file for detailed debugging
        log_file = os.path.expanduser(f'~/{name}_debug.log')
        file_handler = logging.FileHandler(log_file, mode='w')
        file_formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        file_handler.setFormatter(file_formatter)
        logger.addHandler(file_handler)
        
        # Log startup information
        logger.info(f"Logger {name} initialized, logging to {log_file}")
        logger.info(f"Python version: {sys.version}")
        logger.info(f"Environment: {os.environ.get('ROS_DISTRO', 'ROS_DISTRO not set')}")
    
    return logger
