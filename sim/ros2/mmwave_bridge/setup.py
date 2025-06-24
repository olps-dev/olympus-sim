#!/usr/bin/env python3
"""
Setup script for mmWave ROS2 Bridge package
"""

from setuptools import setup, find_packages
from mmwave_bridge.version import __version__

setup(
    name="mmwave_bridge",
    version=__version__,
    description="Bridge between Gazebo mmWave sensor data and ROS2",
    author="Olympus Simulation Team",
    packages=find_packages(),
    install_requires=[
        "numpy",
    ],
    classifiers=[
        "Development Status :: 4 - Beta",
        "Intended Audience :: Developers",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
    ],
    python_requires=">=3.8",
)
