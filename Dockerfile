# Use the official Ubuntu image as the base
FROM python:3.10

# Set the environment variables
ENV DEBIAN_FRONTEND=noninteractive

# Update and install necessary dependencies
RUN apt-get update &&  \
    apt-get install -y \
    git \
    wget \
    unzip \
    && rm -rf /var/lib/apt/lists/*

# Create a user to avoid running as root
RUN useradd -ms /bin/bash carla
RUN echo "carla ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers

# Switch to the new user
USER carla
WORKDIR /home/carla

# Install CARLA
RUN wget https://tiny.carla.org/carla-0-9-15-linux -O CARLA_0.9.15.tar.gz\
    && tar -xvzf CARLA_0.9.15.tar.gz \
    && rm CARLA_0.9.15.tar.gz

# Expose any necessary ports (adjust based on needs, e.g., CARLA uses 2000-2002)
EXPOSE 2000-2002

# Clone the repository into /home/carla directory
RUN git clone https://github.com/tudo-aqua/stars-export-carla.git stars-export-carla

# Set the working directory
WORKDIR /home/carla/stars-export-carla

ENV PATH="/home/carla/stars-export-carla/:$PATH"

# Install required Python dependencies (if any) for the repository
# (adjust this line based on the repository's specific requirements)
RUN pip3 install -r requirements.txt || true

# Start bash by default
CMD ["/bin/bash"]