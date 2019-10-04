# EXPLANATION OF PACKAGE

The rf_communication package is made for the 433MHz radio module, to be used on the Raspberry Pi. The 433Utils scripts were rewritten for the purpose of posting the received messages and sending messages, so in essence we don't need that library. The rc-switch library is used for interfacing with the 433MHz module, via the GPIO pins. The GPIO pins that are used for sending and receiving can be varied per user, so the pin number should be changed accordingly in the codesend.cpp and RFSniffer.cpp (PIN variable), more details on the pin numbers here: projects.drogon.net/raspberry-pi/wiringpi/pins/.

For this ROS package, multiple libraries were used:
- 433Utils library, which includes the rc-switch library (github.com/ninjablocks/433Utils and github.com/sui77/rc-switch)
- WiringPi library, which is used by the rc-switch library (projects.drogon.net/wiringPi)
- spdlog library, for logging (github.com/gabime/spdlog)
- General internal libraries in ROS and C++. (e.g. iostream, sstream)
- Gnome-termimal 


# INSTALLATION INSTRUCTIONS

- wiringPi library should be installed globally. This can simply be done with "sudo apt install wiringpi" (see projects.drogon.net/wiringPi)

- Gnome-terminal should be installed globally, with "sudo apt install gnome-terminal"

- To check the basic workings of the wiringPi library and the 433Utils library, install the 433Utils library separate first, make and run it. This should give you a basic understanding of the workings and it will show if the hardware is working properly.

- Once the previous step is done, clone the package to your local catkin workspace, run "source ./devel/setup.bash" to source catkin, and run "catkin_make" to make our package. 

- If errors:
	- If there are import problems with the rc-switch library, this could be downloaded separately and moved to the include folder in the ros package (maybe it is system dependent).
	- If you get an error such as "Undefined reference to <name of class>::<name of function>", this might be due to wiringPi not being installed properly (see which classes could not be found). It could also be due to the rc-switch library, try and download rc-switch library yourself to see if is a build problem.  

- If succesfull 
	- source the workspace and run "roscore"
	- run "sudo -i" and navigate to the catkin workspace again
	- run "rosrun rf_communication RFSniffer", this should run a few tests and then wait for an incoming message
	- in a new windown, run "sudo -i", navigate to catkin workspace again and run "rosrun rf_communication send_message.py" (YET TO BE INTEGRATED)
	- This will send messages which will be received by RFSniffer, which should be registered and published to the rostopic


