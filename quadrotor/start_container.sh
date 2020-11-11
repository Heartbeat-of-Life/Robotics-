IMG_NAME=ros2:simulation
CONTAINER_NAME=robotics_sandbox
#export GAZEBO_PLUGIN_PATH=$(pwd)/gazebo/plugins/plugin_tutorial/build
#export GAZEBO_MODEL_PATH=$(pwd)/gazebo/models
#export GAZEBO_RESOURCE_PATH=$(pwd)/gazebo/plugins/velodyne_plugin

if [[ -z $(docker ps --filter "name=$CONTAINER_NAME" | grep $CONTAINER_NAME) ]]
then


	ARGS=(" -ti
		--rm
		--user $UID:$GID
		-v /etc/passwd:/etc/passwd
		-v /etc/sudoers:/etc/sudoers:ro
		-v /etc/group:/etc/group:ro
		-v /etc/shadow:/etc/shadow:ro
		-v /tmp/.X11-unix:/tmp/.X11-unix
		-v /run/user/$UID:/run/user/$UID
		-v /dev:/dev
		--name $CONTAINER_NAME
		-w $(pwd)
		-e XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR
		-e TERM=$TERM
		-e DISPLAY=unix$DISPLAY
		-e GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH
		-e GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH
		-e GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH
		--mount type=bind,source=/home,target=/home")

	echo "Starting new docker container"
	docker run --rm  $ARGS $IMG_NAME $SHELL 
else
	echo "Docker container already running. I will attach to running container"
ARGS=(" -ti
	--user $UID:$GID
	-w $(pwd)
	-e XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR
	-e TERM=$TERM
	-e DISPLAY=unix$DISPLAY
	-e GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH")
	docker exec $ARGS $CONTAINER_NAME /usr/bin/ros-entrypoint.sh  $SHELL 

fi

