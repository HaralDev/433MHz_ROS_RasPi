#!/usr/bin/env python2

import os

def desktop_path():
	username = os.environ["USER"]
	if username == "root":
		sudo_user = os.environ["SUDO_USER"]
		path = "/home/%s/Desktop" % sudo_user
	else:
		path = "~/Desktop"

	return path
