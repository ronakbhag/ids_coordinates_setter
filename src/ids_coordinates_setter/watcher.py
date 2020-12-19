import os
import time
from watchdog.observers import Observer
from watchdog.events import PatternMatchingEventHandler
from PIL import Image
import rospy


class ImageWatcher:
	"""
	Class which will be monitoring new files on a given directory 
	Reacts on new file creation: Send's new image path string to an action
	"""

	def __init__(self, src_path, action_client):

		# Monitoring active only for a given set of file extensions
		patterns = "*.bmp;*.jpg;*.png"
		ignore_patterns = ""

		# ignore directory creation
		ignore_directories = True

		# Match Case
		case_sensitive = True

		# set path to monitor
		self.__src_path = src_path

		# Creates handler based on previous configurations
		self.__event_handler = PatternMatchingEventHandler(patterns, ignore_patterns, ignore_directories, case_sensitive)
		self.__observer = Observer()
		self.__action_client = action_client


	def on_created(self, event):
		"""
		callback method for when a new file is detected
		"""
		print("new image, {} has been created!".format(event.src_path))
		# get time when image was created:
		current_milli_time = int(round(time.time() * 1000))

		# convert BMP image to JPG format block:
		basename = os.path.basename(event.src_path)

		# remove file extension
		img_name = os.path.splitext(basename)[0]

		#Read image
		try:
			img = Image.open(event.src_path)
			new_img = img

			# Make dir for storing jpg images, if it doesnt exist already
			newdir = os.path.join(os.path.dirname(event.src_path), "jpeg_converted")

			if not os.path.exists(newdir):
				os.mkdir(newdir)

			# Save jpeg image:
			new_img_abs_path = (newdir + "/" + img_name + ".jpeg")
			new_img.save(new_img_abs_path, 'jpeg')
		except Exception as e:
			rospy.logerr("Could not convert IDS Camera BMP to JPG. Reason: " + str(e))
			return

		self.__action_client.call_server(new_img_abs_path, current_milli_time)


	def run(self):
		"""
		Run logic
		"""
		
		self.__event_handler.on_created = self.on_created
		path = self.__src_path
		
		# Creates and starts observer
		self.__observer.schedule(self.__event_handler, path, recursive=False)
		self.__observer.start()
	
		# run indefinitely:
		# try:
		# 	while True:
		# 			time.sleep(.100)
		# except KeyboardInterrupt:
		# 		self.__observer.stop()
		# 		self.__observer.join()

	def stop(self):
		"""
		Stop logic
		"""
		self.__observer.stop()
		self.__observer.join()

# if __name__ == '__main__':
#
# 	ImageWatcher("/home/ricardo/Pictures", None).run()

