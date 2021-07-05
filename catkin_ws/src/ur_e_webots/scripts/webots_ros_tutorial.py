#!/home/kist-ubuntu/anaconda3/bin/python

""" Code author: DoranLyong 
    

    [Reference]:
    Go to the following links to check the specifications and parameters of the devices. 

        - Camera : https://cyberbotics.com/doc/reference/camera?tab-language=ros
        - Range_finder : https://cyberbotics.com/doc/guide/range-finder-sensors
        - UR10_e_Robots : https://cyberbotics.com/doc/guide/ure
"""
import sys 

import cv2

import rospy 
import rospkg
from std_msgs.msg import Float64
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import message_filters



from webots_ros.srv import (set_int, 
                            set_intRequest, 
                            get_int)
from rosgraph_msgs.msg import Clock


#%%
bridge = CvBridge()





#%%
# ================================================================= #
#                          Visual sensors                           #
# ================================================================= #


def Show_VisualSensing_topics(imgMsg, depthMsg):
    # (ref) https://cyberbotics.com/doc/reference/camera?tab-language=ros#camera-functions
    # (ref) https://cyberbotics.com/doc/reference/rangefinder?tab-language=ros#wb_range_finder_get_range_image
    
    try:
        cv_img = bridge.imgmsg_to_cv2(imgMsg, "passthrough")
        cv_depth = bridge.imgmsg_to_cv2(depthMsg, "passthrough")
    except CvBridgeError as e:
        print(e)

#    cv2.normalize(cv_depth, cv_depth, 0, 1, cv2.NORM_MINMAX)  # (ref) https://answers.ros.org/question/219029/getting-depth-information-from-point-using-python/	

    cv2.imshow("webots_camera_view", cv_img)
    cv2.imshow("webots_depth_view", cv_depth)
    cv2.waitKey(1)	
    




#%%
# ================================================================= #
#                            UR10 Robots                            #
# ================================================================= #


    

    






#%%
if __name__ =='__main__':



    """ Initialization 
    """
    rospy.init_node('UR10_Webots_World_Sensing_and_Control', anonymous=False)  # node initialize fist )


    """ RGB / Depth sensors 
    """   
    rgb_sub =  message_filters.Subscriber("/CAM/camera/image", Image)
    depth_sub = message_filters.Subscriber("/CAM/range_finder/range_image", Image,)
    
    visual_data = message_filters.ApproximateTimeSynchronizer([rgb_sub, depth_sub], 10, 0.1, allow_headerless=True)  # (ref)http://wiki.ros.org/message_filters
    visual_data.registerCallback(Show_VisualSensing_topics)
    
    



    """ UR10 controller publish
    """








    try:
        rospy.spin()

    except KeyboardInterrupt:
        print("Shutting down...")
        
    cv2.destroyAllWindows()

    

