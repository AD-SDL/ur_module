sudo apt install python3-colcon-common-extensions python3-vcstool
mkdir -p ~/ur5_ws/src
cd ~/ur5_ws/src
git clone https://github.com/AD-SDL/henry_module.git
cd ~/ur5_ws
rosdep update
rosdep install --ignore-src --from-paths src -y -r
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

cd src/henry_module
git clone https://github.com/SintefManufacturing/python-urx.git
cd python-urx
pip install .