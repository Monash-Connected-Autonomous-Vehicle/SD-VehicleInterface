# 🐋 mcav-docker
Contains Dockerfiles and a run script.

The `run.sh` script builds and runs the image and container.

The `sd_vehicle_interface` directory is mounted as a volume so it can be accessed from inside and outside the container.

Networking and GUIs should work as normal inside and outside.

## Requirements
Docker: `sudo apt-get install docker.io`

## How to use

### First run
This will build the image if it doesn't exist and enter the container automatically.

- `cd sd_vehicle_interface`
- `docker/run.sh`

### Rebuild the image
- `docker/run.sh build`

### Create a new container from the most recently built image
- `docker/run.sh rm` (removes the existing container)
- `docker/run.sh`
