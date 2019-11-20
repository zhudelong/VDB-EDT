# openvdb_catkin

### A catkin wrapper for the OpenVDB Library from Dreamworks. Enables isolated build of library within a ROS workspace.

#### Dependencies for OpenVDB and the Catkin wrapper
1. `sudo apt-get install libglfw3-dev libblosc-dev libopenexr-dev`
2. `sudo apt-get install liblog4cplus-dev`

#### Installation Steps
1. This is the first step in setting up an individual build of a voxmap workspace. Follow this readme only if you want to set up a separate workspace for testing out voxmap functionalities. 
2. This is because all the relevant packages can be installed through the rosinstall file present in the Planning_workspace package. Use that rosinstall file if you want to use voxmap with the planner and (or) simulation. 
3. Assuming you have already cloned openvdb_catkin into a ROS workspace.
4. `cd /path/to/workspace/src/`
5. `ln -s openvdb_catkin/rosinstall/voxmap.rosinstall .rosinstall`
6. `wstool info`
7. `wstool up`
8. The step above should clone tictoc, base_main_class, catkin_simple, voxmap into the src folder.
9. `catkin build`
10. `source /path/to/workspace/devel/setup.bash`
11. You're all set to run the voxmap node.
12. `roslaunch voxmap voxmap_node_ground.launch`  


#### Additional Information
1. This is a fork from the original github repo from ETHZ at `https://github.com/ethz-asl/openvdb_catkin` . All credits to them for coming up with this.
2. Has been modified to install OpenVDB Ver 5.2.0. Files are downloaded to `path/to/ros/workspace/build/openvdb_catkin/src/openvdb_src`
3. If you want to install an older version of the library, try commit `2dac2d1` (OpenVDB ver 3.0.0).
4. If you want to change the library version to be installed, please make sure you list out all the corresponding .cc source files (for that library version) in `openvdb_catkin/cmake/CMakeLists.txt`

Contact:

Rohit Garg 
rg1@cmu.edu

Delong Zhu

zhudelong@link.cuhk.edu.hk

