#!/usr/bin/env python
import logging
import os
import time

main_file = os.environ.get("MAIN_SEND_MESSAGE_FILE", os.path.basename(__file__).strip(".py"))
logger = logging.getLogger(main_file)


def status_messages(clover_id):
    """Creates the list of raw messages, without checksum, to be send, regarding the status of the 
    Clover.
    """
    logger.debug("Starting function")

    message_time = time.strftime('%M%S')    # Time, for example 11:38:29 becomes 3829, after 10 seconds the message is discared anyway
    rel_pos_x = "1090"      # 10.90m is 1090, x-axis relative position of another rover, just an example    
    rel_pos_y = "0520"      # 5.20m is 0520, y-axis ""

    raw_messages = [message_time, rel_pos_x, rel_pos_y]
    total_hash = total_msg_hash(raw_messages)
    raw_messages.append(total_hash)

    logger.debug("Composing a total of %d messages", len(raw_messages))

    messages = []       # Empty list for composed messages (with hash and everything)

    for i, msg in enumerate(raw_messages, 1):
        hash_msg = sum_hash(msg)		# hash of the message

        msg_send = format_message(message_nr=i, clover_id=clover_id, hash=hash_msg, message=msg)
        logger.info("Adding message to message list: %s", msg_send)
        messages.append(msg_send)

    logger.debug("Composed a total of %d messages", len(messages))
    return messages

def total_msg_hash(message_list):
	logger.debug("Starting function")

	# Get the length of the message, make it length 2 with zfill
	message_length = len(message_list) + 1
	length_string = str(message_length).zfill(2)
	logger.debug("Full message length will be %s", length_string)

	# Get the full message string
	msg_string = "".join(tuple(message_list))
	logger.debug("Full message string is: %s", msg_string)

	# Loop over the string to get the total sum
	total = 0
	for ind, number in enumerate(msg_string, 1):
		logger.debug("Current hash is: %d", total)
		total += int(number) * ind
    
	# Get the total hash, and the mod to make it of length 2
	logger.debug("Total hash is: %d", total)
	total_mod = total % 100
	logger.debug("Total hash with mod is: %d", total)
	hash_string = str(total_mod).zfill(2)
	
	# Format the total string
	total_string = ''.join((length_string, hash_string))
	assert len(total_string) == 4

	logger.debug("Total hash formatted as: %s", total_string)

	return total_string



def format_message(message_nr, clover_id, hash, message):
	logger.debug("Starting function")

	formatted_message = ''.join((str(message_nr), str(clover_id), hash, message))

	logger.debug("Formatted message as: %s", formatted_message)
	assert len(formatted_message) == 7
	return formatted_message


def sum_hash(message):
    """
    Creates the hash for a message, which corresponds to the sum of all the numbers with modulo 10. 
    """
    logger.debug("Starting function")
    total = 0					# Preset the total to 0

    logger.debug("Calculating hash of message: %s", message)
    for number in message:		# Loop over all the numbers in the message 
        total += int(number)	# and sum them.

    total_calculated = total % 10

    logger.debug("Hash calculated: %d", total_calculated)


    if 0 <= total_calculated <= 9:			    # If the sum is single digit, we still want to get 
        hash_string = str(total_calculated)	    # a 0 in front of it.
        logger.debug("Hash returned as: %s", hash_string)
        return hash_string


    else:						                # Otherwise, we will raise a value error
        logger.warning("Hash seems to lie outside the 0-9 range")
        raise ValueError



def test_sum_hash():
    logger.debug("Starting function")
    
    check_str = "6450"
    assert type(sum_hash(check_str)) is str
    if(sum_hash(check_str) != "5"):
        logger.info("Hash for %s is %s and seems to be incorrect" % check_str, sum_hash(check_str))
        return False
    
    check_str = "6454"
    if(sum_hash(check_str) != "9"):
        logger.info("Hash for %s is %s and seems to be incorrect" % check_str, sum_hash(check_str))
        return False

    check_str = "0000"
    if(sum_hash(check_str) != "0"):
        logger.info("Hash for %s is %s and seems to be incorrect" % check_str, sum_hash(check_str))
        return False

    check_str = "0091"
    if(sum_hash(check_str) != "0"):
        logger.info("Hash for %s is %s and seems to be incorrect" % check_str, sum_hash(check_str))
        return False

    check_str = "0090"
    if(sum_hash(check_str) != "9"):
        logger.info("Hash for %s is %s and seems to be incorrect" % check_str, sum_hash(check_str))
        return False

    check_str = "1099"
    if(sum_hash(check_str) != "9"):
        logger.info("Hash for %s is %s and seems to be incorrect" % check_str, sum_hash(check_str))
        return False

    check_str = "1699"
    if(sum_hash(check_str) != "5"):
        logger.info("Hash for %s is %s and seems to be incorrect" % check_str, sum_hash(check_str))
        return False
    
    else:
        return True

if __name__ == "__main__":
    from custom_logger import setup_logging
    logger = setup_logging("test_send_message")
    test_sum_hash()
