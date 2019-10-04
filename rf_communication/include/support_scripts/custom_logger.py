
import logging
import os 
from time import strftime
from support_scripts.get_desktop import desktop_path


def setup_logging(logname="RF_logger"):
    # Mostly from https://www.programcreek.com/python/example/192/logging.Formatter

    desktop_pth = desktop_path()

    # Format
    format_str = '%(filename)15s - %(funcName)-15s > %(asctime)s - %(levelname)s: %(message)s'     # Basic format
    date_format = '%Y-%m-%d %H:%M:%S'                           # Date format
    formatter = logging.Formatter(format_str, date_format)      # Create formatter

    # Create logger
    logger = logging.getLogger(logname)
    logger.setLevel(logging.DEBUG)

    # Create stream handler
    stream_handler = logging.StreamHandler()
    stream_handler.setFormatter(formatter)
    stream_handler.setLevel(logging.INFO)
    logger.addHandler(stream_handler)

    # Create file handler
    date_time = strftime('%y_%m_%d_%H_%M_%S')                   # Date time, which will be the name of the log
    dir_path = "%s/ros_comms_logs/" % (desktop_pth)        # Directory on desktop where logs will be stored
    try:
        os.mkdir(dir_path)                                   # Make directory if it is not there
        logger.info("Logging in file: %s" % dir_path)

    except Exception as exc:
        logger.exception(exc)
        raise exc

    finally:
        file_logname = "%s/%s_%s.log" % (dir_path, date_time, logname)  # Create filename for log
        fh = logging.FileHandler(file_logname)  # Create handler
        fh.setFormatter(formatter)  # Set format
        fh.setLevel(logging.DEBUG)  # Set level
        logger.addHandler(fh)  # Add handler to main logger
        logger.info("Logging setup complete")
        return logger
