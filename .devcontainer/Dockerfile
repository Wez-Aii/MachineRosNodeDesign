# Use the base Ros humble image
FROM ros:humble

# Install the required packages
RUN apt-get update && \
    apt-get install -y python3-pip && \
    apt-get clean

# Set environment variable
ENV ROS_DOMAIN_ID=8

# Set working dir
WORKDIR /workspace

# Copy application dir
COPY requirements.txt /workspace
RUN pip3 install -r requirements.txt

# append commands to .bashrc
RUN echo 'source /ros_entrypoint.sh' >> /root/.bashrc


# # Start the contianer with a command
# CMD ["/bin/bash", "-c", "/ros_entrypoint.sh"]