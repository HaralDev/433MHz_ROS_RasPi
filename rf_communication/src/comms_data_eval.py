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
	from support_scripts.get_desktop import desktop_path
    
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
	<clover_name>:
		<msg>:  {
		    received_nr : <number of times received since deletion>,
		    genesis: <first msg added>
		    }
}
"""

# preset empty dictionary to keep the messages
clover_message_dict = {}

# Preset variables for the data files
date_time = time.strftime('%y_%m_%d_%H_%M_%S')      # Date time, which will be the name of the log
desktop_path = desktop_path()						# get desktop path
data_folder = "%s/communication_data" % desktop_path# Log to desktop in a folder
data_filename = "%s.csv" %   date_time              # Standard file format where logs will be stored, more will be prepended in write_to_csv

# Headers for the csv files
single_msg_csv_hdr = ["time_first_message", "times_received", "clovername"]
full_msg_csv_hdr = ["time_received", "received","clovername"]


try:
	os.mkdir(data_folder)
except OSError:
	pass

logging.info("Creating dataset in folder: %s" % data_folder)

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
        rospy.Subscriber(rostopic_name, String, callback)
    except Exception as exc:
        logger.exception(exc)
        raise
	
	logger.info("Node setup, now listening for messages")
    # keeps python from exiting until this node is stopped
    rospy.spin()

def delete_old_messages(time_now):
    # Check dictionary for messages older than 5 seconds, and delete these.
    #https://realpython.com/iterate-through-dictionary-python/
    logger.info("Searching old messages to delete")

    dict_length = len(clover_message_dict)

    logger.info("Dictionary now has {0} items, which means {0} rovers have recently sent their messages.".format(dict_length))

    for clovername, single_clover_message in clover_message_dict.items():
        # If the clover_message_dict is not empty, do the following
        if dict_length > 0:
            print single_clover_message
            # Loop over dict
            for message, message_details in single_clover_message.items():
		        time_expired = time_now - message_details["genesis"]

		        # A message is currently send from the send_node every 3 seconds,
		        # and the message sending process takes less than 1 seconds, therefore 
		        # wait for 2 seconds to delete the key
		        if time_expired > 2:

					# Before deleting the old message, first write the message data to
					# the csv
					capture_datapoint(message, message_details, clovername)
				
					clover_message_dict.pop(message)
					logging.info("Deleted message %s" % message)


    # If it is empty, there is nothing to delete
	else:
		pass

def capture_single_msg_datapoint(message, message_details, clovername):

	file_name_part = "clover%s_singlemessage" % message[1]
	row = [data["genesis"], data["received_nr"], clovername]
	write_to_csv(row, file_name_part)



def capture_full_msg_datapoint(message, time_now, clovername):

	# Also make a file where to post the data if the full message 
	# was received
	clovername = "clover%s" % message[1]
	file_name_part = "%s_fullmessage" % clovername
	data = [time_now, 1, clovername]
	write_to_csv(data, file_name_part, header = full_msg_csv_hdr)
	



def increment_receive(data, time_now):
    """
    Function increments the proper message by one to indicate another receive of
    the same message
    """
    logger.info("Incrementing message received %s" % data.data)
    message = data.data
    clover_name = "clover%s" % message[1]

    # If message is already there, increment by l
    if clover_name in clover_message_dict:
        if message in clover_message_dict[clover_name]:
            clover_message_dict[clover_name][message]["received_nr"] += 1
        else:   
            clover_message_dict[clover_name][message] = {}
            clover_message_dict[clover_name][message]["received_nr"] = 1
            clover_message_dict[clover_name][message]["genesis"] = 1

    # otherwise set to 1 and define genesis time
    else:
        msg_dict = {}
        msg_dict["received_nr"] = 1
        msg_dict["genesis"] = time_now
        clover_message_dict[clover_name] = {}
        clover_message_dict[clover_name][message] = msg_dict

def write_to_csv(row, file_name_part, header=single_msg_csv_hdr):
    """
    Writes the data to a csv file
    """
    logger.info("Writing to csv")

    filename = "%s/rfdata_%s_%s" % (data_folder, file_name_part, data_filename)

    if os.path.exists(filename):
        logger.info("File exists, writing to it")
        with open(filename, 'a') as csvFile:
            writer = csv.writer(csvFile)
            writer.writerow(row)
        
    else:
        logger.info("File does not exist, create and write to it")
        with open(filename, 'w') as csvFile:
            writer = csv.writer(csvFile)
            writer.writerow(header)
            writer.writerow(row)

def callback(data):
	"""
	Function is initiated when data comes in. So we want to process 
	the data from this function.
	"""
	logger.info("Data incoming")

	# Retrieve time of message incoming, if older than 5 seconds
	time_now = time.time()

	increment_receive(data, time_now)

	delete_old_messages(time_now)

	

    

if __name__ == "__main__":
    listener()
