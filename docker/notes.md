
## Docker for Aerial Inspection Planners

**Note: If you did not get this code from the Sandia GitLab, will need to use the external.Dockerfile as the default Dockerfile includes the necessary proxies and certificates for Sandia use.**

### To Build Docker Image
Run in the docker folder
```sh
docker build -t aerial-inspection .
```

### To run Docker container
Run one of the following commands **at the top of the workspace** so it is mounts the workspace correctly.

Ubuntu/WSL command:
```sh
docker run -it --rm \
    --name aerial-inspection-docker \
	--env="DISPLAY" \
	--env="QT_X11_NO_MITSHM=1" \
	--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
	--mount type=bind,source="$(pwd)",target=/inspection_ws aerial-inspection
```
Windows command:
```sh
docker run -it --rm --name aerial-inspection-docker --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --mount type=bind,source="$(pwd)",target=/inspection_ws aerial-inspection
```

## Getting RViz to show up
If you are on windows, I haven't figured out how to get rviz to show up. Sorry! If you are using WSL, then you should be good to go.

If on ubuntu, go to the next step for rviz to work.

To get the xhost to work on ubuntu
```sh
export containerId=$(docker ps -l -q)
xhost +local:`docker inspect --format='{{ .Config.Hostname }}' $containerId`
docker start $containerId
```

## Other
To join a running container:
```sh
docker exec -it aerial-inspection-docker /bin/bash
```

### If you want to develop in vscode
1. Start container
2. Join in vscode
3. Build code and launch package

Or can just not remove the container and start it up from vscode. You can do this by removing the `--rm` flag on the docker run command.
