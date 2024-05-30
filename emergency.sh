#!/bin/bash

# if the directory "env" does not exist, create a virtual environment
if [ ! -d "env" ]; then
	echo "Creating virtual environment..."
	python3 -m venv env

	source env/bin/activate

	echo "Installing dependencies..."
	pip install -r src/requirements.txt > /dev/null
fi

# there shouldnt be a problem calling this multiple times if the virtual environment is already activated
source env/bin/activate

# get the site-packages path by getting pip show setuptools (setuptools always comes with pip)
VENV_PATH=$(pip show setuptools | grep "Location: " | awk '{print $2}')

export PYTHONPATH="$PYTHONPATH:$VENV_PATH"

cd src

CHECKSUM=$(find ros_turtlebot_camera -type f -exec sha256sum {} + | sort -k 2 | sha256sum | awk '{print $1}')

if [ -f "project_checksum.txt" ]; then
	OLD_CHECKSUM=$(cat project_checksum.txt)

	if [ "$CHECKSUM" != "$OLD_CHECKSUM" ]; then
		echo "Checksums do not match. Rebuilding package..."
		echo $CHECKSUM > project_checksum.txt
		colcon build
	else
		echo "Checksums match. Skipping build..."
	fi
else
	echo "Checksum file not found. Rebuilding package..."
	echo $CHECKSUM > project_checksum.txt
	colcon build
fi

source install/setup.bash

ros2 run ros_turtlebot_camera emergency
