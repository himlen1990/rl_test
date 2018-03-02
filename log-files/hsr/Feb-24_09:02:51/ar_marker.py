import rospy
from visualization_msgs.msg import MarkerArray, Marker

def callback(data):
    #rospy.loginfo("got data")
    #print data.id
    if (data.id == 2):
        print data.pose

def ar_listener():
    rospy.init_node("ar_listener")
    rospy.Subscriber("/visualization_marker", Marker, callback)
    rospy.spin()

if __name__ == '__main__':
    ar_listener()
