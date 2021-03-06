#!/usr/bin/env python2

#====================================================
"""

This script listens to the rostopic rfincoming, which contains incoming 
messages received over radio by the 433MHz radio module. Script comms_data_eval.py 
evaluates how many messages were received, and logs the success of the transfer.

"""

#====================================================
#               IMPORTS
try:
	import os
	import logging
	from support_scripts.custom_logger import setup_logging
    
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
    import rospy
    from std_msgs.msg import String
    import time
    import csv
except ImportError as err:
	logger.exception(err)
	raise

except Exception as err:
	logger.exception(err)
	raise


#====================================================
#               SCRIPTS

# Setup a dictionary to post messages in:
"""
more elaborate way of logging all the messages could be:
clover_message_dict = {

    # Schematic
    <msg number>: {
        time:     <epoch time>,
        raw_msg:  <raw message string>,
        msg:      <only the message>,
        msg_nr:   <number of the message>,
        hash:     <hash of the message>,
        clover_nr:<nr of the clover>
    }
    
    # Example
    1 : {
        time:     125739481,
        raw_msg:  "1138574",
        msg:      "8574",
        msg_nr:   1,
        hash:     3,
        clover_nr:1
    }
}

And then to loop over the msg in each of the msg_numbers, to construct the message.

For a simple evaluation we will use:

clover_message_dict = {
    <msg>:  {
        received_nr : <number of times received since deletion>,
        genesis: <first msg added>
        }
}
"""

clover_message_dict = {}
# Create file handler
date_time = time.strftime('%y_%m_%d_%H_%M_%S')      # Date time, which will be the name of the log
data_filename = "%s.csv" %   date_time               # Directory on desktop where logs will be stored

def listener():
    """
    From wiki.ros.org/rospy/Overview/Publishers%20and%20Subscribers
    listen to the rostopic rfincoming
    """
    logger.info("Setting up listener rosnode")
    # Node and rostopic name to listen to
    node_name = "comms_data_eval"
    rostopic_name = "rfincoming"
    try:
        # Setting up node and subscriber
        rospy.init_node(node_name)
        rospy.Subscriber(rostopic_name)
    except Exception as exc:
        logger.exception(exc)
        raise

    # keeps python from exiting until this node is stopped
    rospy.spin()

def delete_old_messages(time_now):
    # Check dictionary for messages older than 5 seconds, and delete these.
    #https://realpython.com/iterate-through-dictionary-python/
    logger.info("Searching old messages to delete")
    
    # If the clover_message_dict is not empty, do the following
    if len(clover_message_dict) > 0:

        # Loop over dict
        for message, message_details in clover_message_dict.items():
            time_expired = time_now - message_details["genesis"]

            # A message is currently send from the send_node every 3 seconds,
            # and the message sending process takes less than 1 seconds, therefore 
            # wait for 2 seconds to delete the key
            if time_expired > 2:

                # Before deleting the old message, first t

                clover_message_dict.pop(message)
                logging.info("Deleted message %s" % message)

    # If it is empty, there is nothing to delete
    else:
        pass

def increment_receive(data, time_now):
    """
    Function increments the proper message by one to indicate another receive of
    the same message
    """
    message = data.data

    # If message is already there, incrementl
    if message in clover_message_dict:
        clover_message_dict[message]["received_nr"] += 1
    
    # otherwise set to 1 and define genesis time
    else:
        msg_dict = {}
        msg_dict["received_nr"] = 1
        msg_dict["genesis"] = time_now

def write_to_csv(data, time_now):
    """
    Writes the data to a csv file
    """
    row = [data.data, time_now]
    if os.path.exists(data_filename):
        with open(data_filename, 'a') as csvFile:
            writer = csv.writer(csvFile)
            writer.writerow(row)
        
    else:
        with open(data_filename, 'w') as csvFile:
            writer = csv.writer(csvFile)
            writer.writerow(row)

def callback(data):
    """
    Function is initiated when data comes in. So we want to process 
    the data from this function.
    """
    logger.info("Data incoming")

    # Retrieve time of message incoming, if older than 5 seconds
    time_now = time.time()

    delete_old_messages(time_now)

    increment_receive(data, time_now)



