FROM ros:jazzy-ros-base
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-sensor-msgs python3-msgpack \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /app
COPY app/ /app/
ENV PYTHONUNBUFFERED=1 PYTHONDONTWRITEBYTECODE=1 ROS_DOMAIN_ID=0
CMD ["/bin/bash", "-c", "source /opt/ros/jazzy/setup.bash && exec python3 -u /app/benchmark.py"]
