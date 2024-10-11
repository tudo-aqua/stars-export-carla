# Use the official Ubuntu image as the base
FROM python:3

RUN pip3 install --upgrade pip

# Client and Database
RUN pip3 install spython

WORKDIR /opt

CMD [ "spython", "recipe", "stars-export-carla.Dockerfile", "stars-export-carla.snowflake" ]