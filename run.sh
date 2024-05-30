#!/bin/bash

# if the directory "env" does not exist, create a virtual environment
if [ ! -d "env" ]; then
	echo "Creating virtual environment..."
	python3 -m venv env
fi

source env/bin/activate

echo "Installing dependencies..."
pip install -r src/requirements.txt > /dev/null

# get the site-packages path by getting pip show setuptools (setuptools always comes with pip)
VENV_PATH=$(pip show setuptools | grep "Location: " | awk '{print $2}')

export PYTHONPATH="$PYTHONPATH:$VENV_PATH"

cd src

echo "Building package..."
colcon build

source install/setup.bash

ros2 run ros_turtlebot_teleop main
