#!/bin/bash

#set -x #Debug
set -e #exit on failure

#Prepare for ros2
export dir=$(PWD)
cd /tmp
wget https://github.com/ros2/ros2/releases/download/release-crystal-20190408/ros2-crystal-20190408-macos-amd64.tar.bz2
wget https://github.com/AcutronicRobotics/moveit_msgs/releases/download/crystal_patch4/moveit_msgs_crystal_patch4.zip
wget https://github.com/AcutronicRobotics/object_recognition_msgs/releases/download/crystal_patch4/object_recognition_msgs_crystal_patch4.zip
wget https://github.com/AcutronicRobotics/octomap_msgs/releases/download/crystal_patch4/octomap_msgs_crystal_patch4.zip
tar jxf ros2-crystal-20190408-macos-amd64.tar.bz2
unzip moveit_msgs_crystal_patch4.zip -d moveit_msgs_crystal_patch4
unzip object_recognition_msgs_crystal_patch4.zip -d object_recognition_msgs_crystal_patch4
unzip octomap_msgs_crystal_patch4.zip -d octomap_msgs_crystal_patch4
cp -r moveit_msgs_crystal_patch4/* ros2-osx/
cp -r object_recognition_msgs_crystal_patch4/* ros2-osx/
cp -r octomap_msgs_crystal_patch4/* ros2-osx/
# Remove tf2_eigen
find ros2-osx/ -name tf2_eigen | xargs rm -rf
find ros2-osx/ -name resource_retriever | xargs rm -rf
