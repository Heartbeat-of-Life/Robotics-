IMG_NAME=ros:base
CONTAINER_NAME=robotics_sandbox

if [[ -z $(docker ps --filter "name=$CONTAINER_NAME" | grep $CONTAINER_NAME) ]]
then

	ARGS=(" -ti
		--rm
		--user $UID:$GID
		-v /etc/passwd:/etc/passwd
		-e TERM=$TERM
		--name $CONTAINER_NAME
		--mount type=bind,source=/home,target=/home")


	echo "Starting new docker container"
	docker run $ARGS $IMG_NAME $SHELL 
fi

	docker exec -ti $CONTAINER_NAME /usr/bin/ros-entrypoint.sh $SHELL
