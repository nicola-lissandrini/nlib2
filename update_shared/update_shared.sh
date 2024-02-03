#!/bin/bash

package_dir=
current_dir=$(pwd)
found=false

while [ "$current_dir" != "/" ]; do
	current_dir=$(pwd)
	
	if [ -f "package.xml" ]; then
		package_dir=$current_dir
		found=true
		break
	else
		cd ..
	fi
done

if ! $found; then
	echo "Command was not run on a valid package"
	exit -1
fi

package_name=$(basename $package_dir)
echo "Package found: $package_name"
pwd

workspace_dir=
workspace_found=false

while [ "$current_dir" != "/" ]; do
	current_dir=$(pwd)

	if [ -d "install" ]; then
		workspace_dir=$current_dir 
		workspace_found=true
		break;
	else
		cd ..
	fi
done

if ! $workspace_found; then
	echo "No valid workspaces found."
	exit -2
fi 

echo "Workspace found: $workspace_dir"
source /opt/ros/iron/setup.bash
source ./install/setup.bash

echo "Running"
echo "colcon build --packages-select $package_name --cmake-args -DSHARED_ONLY=ON "

colcon build --packages-select $package_name --cmake-args -DSHARED_ONLY=ON 
