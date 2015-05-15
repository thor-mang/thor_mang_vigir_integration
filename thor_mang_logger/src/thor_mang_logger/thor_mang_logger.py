#!/usr/bin/env python

import subprocess
import os
import signal
import yaml
import time
import datetime

import rospy
import rospkg

class ThorMangLogger(object):

	def __init__(self, config_fn):
		config_content = None
		rp = rospkg.RosPack()
		config_path = os.path.join(rp.get_path('thor_mang_logger'), "config", config_fn + ".yaml")
		with open(config_path, 'r') as config_stream:
			config_content = yaml.load(config_stream)

		timestamp = time.time()
		timestring = datetime.datetime.fromtimestamp(timestamp).strftime('%Y-%m-%d_%H-%M-%S')

		bagfile_path = os.path.expanduser(config_content['bagfile_path'])
		if not os.path.exists(bagfile_path):
			os.makedirs(bagfile_path)
		self.bagfile_name = os.path.join(bagfile_path, timestring + ".bag")
		self.bash_command = ["/bin/bash", "--norc", "-c"]
		self.topics_to_record = config_content['topics_to_record']

		self._bag_process = None



	def start(self):
		bag_command = "rosbag record -O {}".format(self.bagfile_name) + " " +  ' '.join(self.topics_to_record)
		self._bag_process = subprocess.Popen(self.bash_command + [bag_command], stdout=subprocess.PIPE, preexec_fn=os.setsid)
		rospy.loginfo('Recording %d topics to %s' % (len(self.topics_to_record), self.bagfile_name))


	def stop(self):
		rospy.loginfo('Stopping topic recording to %s' % self.bagfile_name)
		try:
			if self._bag_process is not None:
				os.killpg(self._bag_process.pid, signal.SIGINT)
		except Exception as e:
			rospy.logwarn("Unable to kill process %s:\n%s" % (str(self._bag_process.pid), str(e)))