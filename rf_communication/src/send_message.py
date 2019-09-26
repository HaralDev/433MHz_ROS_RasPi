#!/usr/bin/env python2

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
	from subprocess import PIPE
	from time import sleep
except ImportError as err:
	logger.exception(err)
	raise

except Exception as err:
	logger.exception(err)
	raise

if __name__ == "__main__":

	
	# Run tests
	logger.info("Running tests")
	logger.info("Running test_sum_hash().")
	test_valid = test_sum_hash()
	if(not test_valid):
		logger.warn("Test_sum_hash was unsuccesful.")
	else:
		logger.info("Test_sum_hash was succesful.")
	
	# Set id and RF parameters
	clover_id = os.environ.get("CLOVER_ID", str(1))
	codesend_file = os.environ.get("CODESEND_FILE", "./codesend")
	protocol = '2'  # see RCSwitch.cpp lines 70-80
	logger.info("Parameters set. Clover_id: %s, codesend_file: %s, RF Protocol: %s", clover_id, codesend_file, protocol)

	# Set status message
	messages = status_messages(clover_id)

	logger.info("Running for loop to send message.")
	
	for i, msg in enumerate(messages, 1):
		
		logger.info("Sending message %s", msg)
		command = [codesend_file, msg, protocol]
		logger.debug("Sending command: %s", ''.join(tuple(command)))
		
		MyOut = subprocess.Popen(command, 
					stdout=subprocess.PIPE, 
					stderr=subprocess.STDOUT)
		stdout,stderr = MyOut.communicate()
		logger.debug("Stdout: %s", stdout)
		if stderr is not None:
			logger.warn("Stderr: %s", stderr)
		sleep(1)
