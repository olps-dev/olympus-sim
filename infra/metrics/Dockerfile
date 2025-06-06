# Project Olympus - KPI Collection and Analysis Service
FROM python:3.11-slim

# Set working directory
WORKDIR /app

# Install system dependencies
RUN apt-get update && apt-get install -y \
    curl \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies
COPY requirements.txt ./
RUN pip install --no-cache-dir -r requirements.txt

# Copy source code
COPY . /app/

# Create data directory
RUN mkdir -p /data

# Set permissions
RUN chmod +x /app/*.py 2>/dev/null || true

# Default command
CMD ["python", "metrics_collector.py"] 