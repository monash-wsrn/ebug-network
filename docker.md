# Docker Image Building Guide

## Clone Respository
```
git clone https://github.com/MonashRobotics/networked_robotics.git
```

## Checkout into Kelvin's Branch
```
git checkout Kelvin
```

## Download Docker
```
sudo apt install docker.io
```

## Build the Docker Image
```
cd networked_robotics
sudo docker build . -t light_robot:1.0
```
Takes some time to build the docker image, so please be patient.

## Run the Docker Image in a container
```
sudo docker run -it light_robot:1.0
```

## To exit the Docker Container
```
exit
```
