# Docker Image Building Guide

## Clone Respository
```
git clone https://github.com/monash-wsrn/ebug_network.git
```

## Download Docker
```
sudo apt install docker.io
```

## Build the Docker Image
```
cd ebug_network/ebug_ws/src/ebug
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
