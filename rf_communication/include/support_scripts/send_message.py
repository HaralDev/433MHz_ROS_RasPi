#!/usr/bin/env python2

#====================================
# 		IMPORTS AND LOGGING SETUP

try:
	import os
	import logging
	from custom_logger import setup_logging
except ImportError as err:
	logging.exception(err)
	raise

except Exception as err:
	logging.exception(err)
	raise
else:
	main_file = os.path.basename(__file__).strip(".py")
	os.environ["MAIN_SEND_MESSAGE_FILE"] = main_file
	logger = setup_logging(main_file)

try:
    from send_support import *
    import subprocess
    import rospy
    from subprocess import PIPE
    from time import sleep
except ImportError as err:
	logger.exception(err)
	raise

except Exception as err:
	logger.exception(err)
	raise


#=================================
# 		MAIN PROGRAMME

def main_programme():
    try:
        #-------------------------------------------------
        # Run tests
        logger.setLevel(logging.DEBUG)
        logger.info("Running tests")
        logger.info("Running test_sum_hash().")
        test_valid = test_sum_hash()
        if(not test_valid):
            logger.warn("Test_sum_hash was unsuccesful.")
        else:
            logger.info("Test_sum_hash was succesful.")


        #-------------------------------------------------
        # Set id and RF parameters
        clover_id = rospy.get_param("/CLOVER_NR")
        codesend_file = os.environ.get("CODESEND_FILE", "codesend")
        protocol = '0'  # see RCSwitch.cpp lines 70-80
        logger.info("Parameters set. Clover_id: %s, codesend_file: %s, RF Protocol: %s", clover_id, codesend_file, protocol)


        #-------------------------------------------------
        # Set status message
        try:
            messages = status_messages(clover_id)
        except Exception as exc:
            logger.exception(exc)
            sleep(.1)
            raise exc
        #-------------------------------------------------
        # Run the loop to send messages every 1 seconds
        logger.info("Running for loop to send message.")
        big_sleeptime = 1.8		# [seconds] Time between each message set
        small_sleeptime = 0.3		# [seconds] Time between each message, this worked with a delay of 1, so now reducing to see performance

        while(1):
	        for i, msg in enumerate(messages, 1):
		        logger.info("Sending message %s", msg)

		        # Compose whole command to send to codesend
		        # Run the rosnode to send the message, if error "OSError: [Errno 2] No such file or directory"
		        # is returned, this might mean that the workspace is not yet made. It at least means that it
		        # can not find an executable file named "codesend"
		        command = ["rosrun", "rf_communication", 'codesend', msg, protocol]
		        logger.info("Sending command: %s", ' '.join(tuple(command)))
	
		        # Run command and get output

		        MyOut = subprocess.Popen(command, 
					        stdout=subprocess.PIPE, 
					        stderr=subprocess.STDOUT)
		        stdout,stderr = MyOut.communicate()

		        # Log the output
		        logger.debug("Stdout: %s", stdout)

		        # If an error is returned, log this with warn level
		        if stderr is not None:
			        logger.warn("Stderr: %s", stderr)
	
		        # Sleep for a small time between each message (to prevent overflow)
		        sleep(small_sleeptime)

	        # Sleep for a longer time between each message
	        sleep(big_sleeptime)
    except Exception as exc:
        logging.warn(exc)
        raise
