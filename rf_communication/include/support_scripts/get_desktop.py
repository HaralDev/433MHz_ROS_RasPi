#!/usr/bin/env python2

import os
import logging
import rospy

def desktop_path():
	username = os.environ["USER"]
	if username == "root":
		sudo_user = os.environ["SUDO_USER"]
		path = "/home/%s/Desktop" % sudo_user
	else:
		path = "~/Desktop"

	return path


def main():
    logging.info("Running get_desktop path, setting dekstop path as ROS param")
    desktop_path_current = desktop_path()
    rospy.set_param("/DESKTOP_PATH", desktop_path_current)
    logging.info("ROS Parameters are: {}", str(rospy.get_param_names()))
    logging.info("Desktop path set to: {}".format(desktop_path_current))
    return
