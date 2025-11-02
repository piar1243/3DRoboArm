In this repository, there are the files for the 3D model of the robotic arm so you can 3D print it, and the code scripts (and external Catkin Workspace) for ROS and the microcontroller. 

Note: build at your own risk, as I’m still refining many errors within the 3D model that I fixed physically and not on the model. I’m also changing the model to add internal wiring. Additionally, I’m still refining the pid system between ROS and Arduino so that the Esp32 microcontroller can be used. Finally, more modular parts are being worked on at this time.

If you would like to build this robotic arm, need additional assistance, or are looking to collaborate for future versions, please feel free to contact me at: franklucci636@gmail.com.

I hope you enjoy the project!

External Code Framework/Library/Function Credit:
Kitamura, Fusion2URDF → aided in conversion of Fusion 360 3D model to URDF
Adafruit, TCA9548A → code function to switch between encoders
Curious Scientist, AS5600 encoder tutorial → AS5600 read angle function
ROS → robotic arm’s framework
MoveIt → robotic arm kinematic motion planning

