docker run -ti --rm --user $UID:$GID -v /etc/passwd:/etc/passwd --mount type=bind,source=/home,target=/home -v /etc:/etc  ros:base /$SHELL

