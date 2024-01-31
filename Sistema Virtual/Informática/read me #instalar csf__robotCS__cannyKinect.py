#instalar csf\_robotCS\_cannyKinect.py
sudo apt-get install python3-pip
sudo apt install python-pip
pip3 install opencv-python

Create pakcage
cd ~/catkin_ws/src
catkin_create_pkg img_processor std_msgs rospy

cd ~/catkin_ws/src/img_processor/src
csf_robotCS_cannyKinect.py

source ~/catkin_ws/devel/setup.bash
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc

roscd img_processor

cd ~/catkin_ws
catkin_make
