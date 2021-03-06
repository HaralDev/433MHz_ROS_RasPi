/*
  RFSniffer

  Usage: ./RFSniffer [<pulseLength>]
  [] = optional

  Hacked from http://code.google.com/p/rc-switch/
  by @justy to provide a handy RF code sniffer

  Adapted more by Harald Luiks in September 2019

  Using logging library spdlog 
  	Repo: https://github.com/gabime/spdlog
	Docs: https://github.com/gabime/spdlog/wiki/1.-QuickStart
	(if install )

*/

#include <rc-switch/RCSwitch.h>
#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <unistd.h> 
#include <iostream>
#include <sstream>
#include <cmath>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/basic_file_sink.h> // support for basic file logging
#include <spdlog/sinks/stdout_color_sinks.h> // or "../stdout_sinks.h" if no colors needed
#include "support_scripts/RFSupport.h"

// ROS, from http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29
#include "ros/ros.h"
#include "std_msgs/String.h"

using namespace std; 

RCSwitch mySwitch;
 

//=================================================
// 						MAIN
int main(int argc, char *argv[]) {

     //              ROS SETUP
    // Need this for parameters
	ros::init(argc, argv, "RFSniffer");
	
	ros::NodeHandle n;

   // Receiving Pin
    int PIN;
    if(!n.getParam("/PIN_IN", PIN)) {
        cerr << "PIN_IN environment variable is not set, please use launch file or set it manually";
        return -1;
    }

    std:string DESKTOP_PATH;
    if(!n.getParam("/DESKTOP_PATH", DESKTOP_PATH)) {
        cerr << "DESKTOP_PATH ros parameter variable is not set, please use launch file or set it manually";
        return -1;
    }

	// 				LOGGING SETUP
	// From: https://github.com/gabime/spdlog/wiki/1.-QuickStart
	try {

        char path [256];
        sprintf(path, "%s/logs/log_RFSniffer.txt", DESKTOP_PATH.c_str());    
        //Sstd:string log_path = "%s/logs/log_RFSniffer.txt" % DESKTOP_PATH;
        auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(path, true);
		file_sink->set_level(spdlog::level::debug);
		
		auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
		console_sink->set_level(spdlog::level::debug);

		spdlog::set_default_logger(std::make_shared<spdlog::logger>("multi_sink", spdlog::sinks_init_list({console_sink, file_sink})));		
		spdlog::set_level(spdlog::level::debug);
    }

    catch (const spdlog::spdlog_ex& ex) {
        std::cout << "Log initialization failed: " << ex.what() << std::endl;
    }

	spdlog::info("Logging successfully setup");
	spdlog::info("Logging with debug level during setup");
	
	
	spdlog::info("============================");
	spdlog::info("Starting RFSniffer programme");
  	

	//---------------------------------------------
	// 				TESTS
	spdlog::info("Run tests");

	if (test_check_msg()==false) {
		spdlog::warn("Testing with test_check_msg() failed, stop programme");
		spdlog::info("============================");
		return -1;
	}
	else if (test_get_int_from_str()==false) {
		spdlog::warn("Testing with test_get_int_from_str() failed, stop programme");
		spdlog::info("============================");
		return -1;
	}
	else {
		spdlog::info("All tests were successful");
		spdlog::info("============================");
	}


	//---------------------------------------------
	//				SETTING UP
	spdlog::info("Setting up parameters and required objects (e.g. wiringPi, ROS)");

    //			LOOP PARAMETERS
	spdlog::info("Setting up loop parameters");
	std::string msg_string;
	std::stringstream ss;
	
	int loopFreq = 20;
	spdlog::info("Looping at '{}' hz.", loopFreq);

	spdlog::info("Loop parameters setup successful");

    //			ROS PARAMETERS
	// From http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29
	spdlog::info("Setting up ROS parameters");

	ros::Publisher chatter_pub = n.advertise<std_msgs::String>("rfincoming", 1000);

	ros::Rate loop_rate(loopFreq);


	//				WIRINGPI
     // This pin is not the first pin on the RPi GPIO header!
     // Consult https://projects.drogon.net/raspberry-pi/wiringpi/pins/
     // for more information.
     spdlog::info("Setting up wiringPi");
	 
	 // SETTING THE RECEIVING PIN
    
	 spdlog::info("Using PIN '{}' as receiving pin", PIN);
     
     if(wiringPiSetup() == -1) {
		spdlog::warn("wiringPiSetup failed, exiting...");
       	return 0;
     }

     int pulseLength = 0;
     if (argv[1] != NULL) pulseLength = atoi(argv[1]);
	 spdlog::info("Using pulseLength: '{}'", pulseLength);

     mySwitch = RCSwitch();
     if (pulseLength != 0) mySwitch.setPulseLength(pulseLength);
     mySwitch.enableReceive(PIN);  // Receiver on interrupt 0 => that is pin #2
     
	spdlog::info("WiringPi setup successful");



	spdlog::info("Setup complete");
	spdlog::info("============================");
    

    
	//---------------------------------------------
	//				MAIN LOOP
    time_t t_now;
	spdlog::info("Running main loop, listening for messages");
    while(ros::ok()) {
        
        // Post message to indicate script is still looking pessage
		if (time(&t_now) % 50 == 0) {
            spdlog::info("Waiting for message on PIN: %d ...", PIN);
        }

      	if (mySwitch.available()) {

			int value = mySwitch.getReceivedValue();		// Receive value

			if (value < 1000000 || value > 10000000) {		// Check value for length first, should be length 7
				spdlog::debug("Received value out of bounds: %d", value);
			} 
			else {    
				std::stringstream ss;		// Create a new stringstream 			
				std_msgs::String msg;		// Create the msg variable

				ss << value;				// Pipe the value in the stringstream

				msg.data = ss.str();		// Assign the string to msg_string variable
				
				
				// 		Check message validity
				if(check_msg(msg.data)) {
					// Publish to rostopic
					// http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29
					spdlog::info("PUBLISHING MESSAGE TO ROSTOPIC");
					chatter_pub.publish(msg);
					ros::spinOnce();

				}
				
				else {
					spdlog::debug("Message invalid: %d", value);// Do nothing if the message is not valid
				}
				
			}
    
        
        mySwitch.resetAvailable();
    }
    loop_rate.sleep();
  
  }
  fflush(stdout);
  exit(0);


}

