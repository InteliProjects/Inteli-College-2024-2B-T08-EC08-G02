# Use a ROS 2 image with Python support as the base
FROM ros:humble-ros-base

# Set the working directory
WORKDIR /code

RUN apt-get purge -y python3-*transforms3d || true \
    && rm -rf /usr/local/lib/python3*/dist-packages/transforms3d*

# Install required system dependencies and TurtleBot3 packages
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    ros-humble-turtlebot3* \
    ros-humble-nav2-bringup \
    ros-humble-navigation2 \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-tf-transformations \
    && rm -rf /var/lib/apt/lists/* \
    espeak

# Upgrade pip and set Python alias
RUN python3 -m pip install --upgrade pip
RUN ln -s /usr/bin/python3 /usr/bin/python

# Copy the requirements file and install additional Python dependencies
COPY ./requirements.txt /code/requirements.txt
RUN pip install --no-cache-dir -r /code/requirements.txt

# Copy the FastAPI application code
COPY ./app /code/app

# Source ROS 2 setup file and run the FastAPI application
CMD ["bash", "-c", "source /opt/ros/humble/setup.bash && export TURTLEBOT3_MODEL=burger && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && export ROS_DOMAIN_ID=117 && uvicorn app.main:app --host 0.0.0.0 --port 8000"]
