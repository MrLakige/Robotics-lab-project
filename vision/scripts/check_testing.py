import rospy
#from gazebo_msgs.msg import ModelStates
from vision.msg import Block

def message_callback(data):
   
        #print("message:")
    print(data)
    #posx = data.posx
    #posy = data.posy
    #posz = data.posz
    #quatx = data.quatx
    #quaty = data.quaty
    #quatz = data.quatz
    #quatw = data.quatz
    #class_id = data.class_id

def message_reader():
    print("start node")
    rospy.init_node("check_reader", anonymous=True)
    print("waiting message")
    #while not rospy.is_shutdown():
    try:
        rospy.Subscriber('/mega_blocks_detections', Block, message_callback)
        #rospy.Subscriber('/gazebo/model_states', ModelStates, message_callback)
        rospy.spin()
    except rospy.ROSException:
        pass


if __name__ =='__main__':
    message_reader()

