
sudo apt install python3-colcon-common-extensions python3-vcstool
rosdep update
rosdep install --ignore-src --from-paths src -y -r
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
git clone https://github.com/SintefManufacturing/python-urx.git
cd python-urx
pip install .