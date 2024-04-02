# ADHIC
Autonomous Disinfectant Holonomic Interactive Cobot

ADHIC is a autonomous holoniomic drive bot which uses a RP Lidar to map its sorroundings and move autonomously. The main moto of the bot is to move in places like hospitals, hotels etc. autonoumously and disinfect the the surfaces like beds, tables, chairs etc. It has a 3 Degree Of Freedom Robotic arm which is having a UV Lamp as end-effector.

Hardware Used:
1. Raspberrypi 4 (8gb)
2. Slamtec RP Lidar A2
3. 3 DC motors and 3 Omni wheels
4. 2 L298N motor drivers
5. 3 Rotory Encoders
6. A suitable battery
7. Acrylic Chassiss

Software Used:
1. ROS(Noetic)
2. Gazebo
3. RVIZ
4. MoveIt
5. Fusion 360
6. Remmina
7. Arduino IDE



Process Flow

The bot is holonomic i.e the three wheels which are seperated with 120 degree angle, so for bot's motion we need a velocity matrix which converts the bot's x,y and z axis speeds into the three individual motor speeds. Three Encoders are attached to each wheel which gives the position of each wheel from its start state.
These encoder values are again put into the inverse matrix of the velocity matrix to get the bot's x,y,theta from the three individual values of the encoder.

These values will be published to the RPI as /odom topic so that the rpi can can create a map of bot's surroundings using the RP-Lidar. This SLAM node will publish /cmd_vel topic which the arduino mega has to subscribe for autonomous motion (Using ros-serial). So the arduino will have a callback function for this subcriber which will give velocity in x,y,theta which will be further converted into the 3 individual speed of wheels.

For creation of the map, first we have to move the bot in the area where map has to be made using teleop_twist_keyboard. Once the map is generated, it is saved and the bot now can move autonomously in the map where the goals are decided. 

So the bot will move to the beds and chairs, wait over there and get the point cloud of the surface using a depth camera. From that point cloud we will get the 3D coordinates where the robotic arm has to be moved. Now the MoveIt which is configured according to the 3 degree of freedom will move its end effector over the point cloud generated. On beds it will move horizontally and disinfect the surface using the UV light attached at the end of the arm. 

Hence, the bot is fully autonomous once the map is generated and the goals are decided!



# AWS Hospital world
https://github.com/aws-robotics/aws-robomaker-hospital-world
