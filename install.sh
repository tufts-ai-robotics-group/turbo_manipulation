git clone https://github.com/atenpas/gpd_ros.git
git clone https://github.com/tufts-ai-robotics-group/turbo_manipulation.git
git clone https://github.com/atenpas/gpd.git

cp turbo_manipulation/gpd_files/gpd/ros_eigen_params.cfg gpd/cfg/
cp turbo_manipulation/gpd_files/gpd/CMakeLists.txt gpd/
cp turbo_manipulation/gpd_files/gpd_ros/ur5.launch gpd_ros/launch/
cp turbo_manipulation/gpd_files/gpd_ros/grasp_detection_node.cpp gpd_ros/src/gpd_ros/

cd gpd
git checkout 6c6f9752b6197bdeffbf861da57ec04f96549148
mkdir build && cd build
cmake ..
sudo make install
cd ..

catkin_make
