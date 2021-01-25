SCRIPT_DIR=${PWD}
INSTALL_DIR=install
if [ ! -d ${INSTALL_DIR} ]; then
	echo "There is no installation available. Please build your Code by using colcon build"
	return;
fi

LIBDIRS="$(dirname $(find ${INSTALL_DIR} -name 'lib*.so'))"

for dir in ${LIBDIRS}; do
	cd ${dir}
	export LD_LIBRARY_PATH=$(pwd):${LD_LIBRARY_PATH}
done

cd $SCRIPT_DIR


ros2 launch sandbox_launch sandbox_launch.py verbose:=true  world:=gazebo/worlds/pendulum.world       #extra_gazebo_args:="--verbose"
