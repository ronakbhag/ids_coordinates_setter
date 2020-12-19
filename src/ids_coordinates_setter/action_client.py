import rospy
import actionlib
from ids_coordinates_setter.msg import ChangeEXIFAction, ChangeEXIFGoal


class ActionClient:
    """
    Class which consumes an action from a Server
    """

    def feedback_cb(self, msg):
        print ('Feedback received:', msg)

    def call_server(self, path, milliseconds):
        print ("sending this path to action: ", path)

		# Create client to consume "change_exif" action
        client = actionlib.SimpleActionClient('change_exif', ChangeEXIFAction)
        client.wait_for_server()

		# Set action goal to target images path
        goal = ChangeEXIFGoal()
        goal.path = path
        goal.timestamp_in_milliseconds = milliseconds
		# Send goal to action server and wait for action result
        client.send_goal(goal, feedback_cb=self.feedback_cb)
        # client.wait_for_result()
        # result = client.get_result()
        # return result

    if __name__ == '__main__':
        try:
            rospy.init_node('action_client')
            result = call_server()
            print ('The result is:', result)
        except rospy.ROSInterruptException as e:
            print ('Something went wrong:', e)
