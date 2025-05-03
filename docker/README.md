# 🐋 mcav-docker
Contains Dockerfiles and a run script.

Docker is an alternative if you don't have a Ubuntu 22 VM.

1. Install docker CLI: https://www.docker.com/get-started/
2. In this folder, run
```bash
docker build -t ros2-vehicle-interface .
```

3. Run the docker image with the options of your choice
Main cmd:
```bash
docker run -it --name ros2-vehicle-interface-container ros2-vehicle-interface
```
Feel free to change the `--name`

Options:
- Remove container after running: `--rm`
- Allow it to access USB device: `--device=<path-tod-device>`

## Basics
#### Exiting container
```bash
exit
```

#### Listing containers
Running containers:
```bash
docker ps
```
- Use `-a` flag for all containers (running or not)

#### Starting and attaching container
Start container if it has been stopped (i.e. not in `docker ps`)
```bash
docker start <container_id_or_name>     
```

Attach to container to start working in that environment.
```bash
docker exec -it <container_id_or_name> bash    
```

#### Container/Image Removal
If the container/image is taking too much space, feel free to remove it 

Container:
1. find id or name using:
```bash
docker ps -a
```
2. Remove forcefully with: 
```bash
docker rm -f <container_id_or_name>
```

Image:
1. find id or name with:
```bash
docker images
```
2. Remove forcefully with: 
```bash
docker rmi -f
```

#### Editing docker files via VSCode from your host machine
Don't like vim nor nano? Use VSCode dev: https://marketplace.visualstudio.com/items/?itemName=ms-vscode-remote.remote-containers

#### tmux
Don't want to open multiple terminals just for docker? Have a look at tmux: https://github.com/tmux/tmux/wiki

#### Rviz2 (experimental)
https://github.com/adeeb10abbas/ros2-docker-dev/tree/master