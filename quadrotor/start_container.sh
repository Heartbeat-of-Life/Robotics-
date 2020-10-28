IMG_NAME=ros:simulation
CONTAINER_NAME=robotics_sandbox

if [[ -z $(docker ps --filter "name=$CONTAINER_NAME" | grep $CONTAINER_NAME) ]]
then

	ARGS=(" -tid
		--rm
		-v /etc/passwd:/etc/passwd
		-v /etc/sudoers:/etc/sudoers:ro
		-v /etc/group:/etc/group:ro
		-v /etc/shadow:/etc/shadow:ro
		-v /tmp/.X11-unix:/tmp/.X11-unix
		-v /run/user/$UID:/run/user/$UID
		--name $CONTAINER_NAME
		-w $(pwd)
		-e XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR
		--mount type=bind,source=/home,target=/home")


	echo "Starting new docker container"
	docker run $ARGS $IMG_NAME /usr/bin/ros-entrypoint.sh  $SHELL 
else
	echo "Docker container already running. I will attach to running container"
fi

	ARGS=(" -ti
		--user $UID:$GID
		-e TERM=$TERM
		-e DISPLAY=unix$DISPLAY
		-w $(pwd)")

		docker exec $ARGS $CONTAINER_NAME /usr/bin/ros-entrypoint.sh  $SHELL 
