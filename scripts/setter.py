#!/usr/bin/env python
import rospy
import subprocess
import signal, os
from mavros_msgs.msg import RCIn
from ids_coordinates_setter.watcher import ImageWatcher
from ids_coordinates_setter.action_client import ActionClient
import shutil
import time

class Setter:
	"""
	Class which Initiates a directory monitoring
	Subscribes a Topic and waits for a specific message which will enable this node logic to start:
		- Executes Camera Script
		- Starts ImageWatcher class
	"""

	def __init__(self):
		rospy.loginfo("Running %s", rospy.get_caller_id())
		self.path = "/"  # I'll the images path from ROS Param
		rospy.Subscriber(rospy.get_param('startupTopic'), RCIn, self.callback)
		self.isrunning = False
		self.script_process = None
		self.image_watcher = None
		self.rc_channel_max = rospy.get_param('trigger_channel_max_val')
		self.rc_channel_min = rospy.get_param('trigger_channel_min_val')

	def make_absolute_paths(self, script_root="", images_root=""):
		new_path_script = script_root
		new_path = images_root
		number = 1
		created_path = False
		while created_path == False:
			new_path = images_root + '/autoc' + str(number) + '/'
			if os.path.exists(new_path):
				number = number + 1
			else:
				try:
					rospy.loginfo('Create path to save images in %s', new_path)
					os.makedirs(new_path)
					new_path_script = new_path + os.path.basename(script_root)
					shutil.copy2(script_root, new_path)
					rospy.loginfo('New path created in %s with script %s',new_path, os.path.basename(script_root))
					created_path = True
					if not os.path.exists(new_path):  # If mkdir not work
						return script_root, images_root
				except Exception as e:
					rospy.logerr('Failed to create new path for image and script. \nError: %s', e.message)
					return script_root, images_root
		return new_path_script, new_path

	def callback(self, data):
		# Run Logic
		try:
			channel_value = data.channels[5]
		except Exception as e:
			return

		if channel_value >= self.rc_channel_max and self.isrunning == False:
			# tell the code our logic is already running
			self.isrunning = True

			# get absolute paths and call camera script thread:
			script_path, img_path = self.make_absolute_paths(script_root=rospy.get_param('~camera_script_root_path'),
															 images_root=rospy.get_param('~image_root_path'))

			# Run external camera script
			self.script_process = subprocess.Popen(script_path, cwd=img_path)

			# Set directory path to monitor based on a rosparam
			self.path = img_path

			# Start Action Client
			self.ac = ActionClient()

			# Start thread for monitoring directory for new images:
			self.image_watcher = ImageWatcher(self.path, self.ac)  # Create Watcher for monitoring a directory
			self.image_watcher.run()

		# Stop Logic
		if channel_value <= self.rc_channel_min and self.isrunning == True:
			self.isrunning = False
			rospy.loginfo("Attempting to kill ids_coordinates_setter image processes...")
			try:
				# os.killpg(os.getpgid(self.script_process.pid), signal.SIGTERM)
				self.script_process.kill()
				time.sleep(30)
				self.image_watcher.stop()
			except Exception as e:
				rospy.logerr("FAILED TO KILL ids_coordinates_setter IMAGE PROCESSES")
				print (e.message)


if __name__ == '__main__':
	rospy.init_node('setter')
	s = Setter()
	rospy.spin()
