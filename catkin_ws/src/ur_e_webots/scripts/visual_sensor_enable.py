#!/usr/bin/python

"""
    (reference for rgb camera) https://cyberbotics.com/doc/reference/camera?tab-language=ros#wb_camera_enable
    (reference for depth map ) https://cyberbotics.com/doc/reference/rangefinder?tab-language=ros#wb_range_finder_enable

"""


import rospy 

from webots_ros.srv import (set_int, 
                            set_intRequest,
                            )


sensor_dict = { 'rgb_sensor' : '/CAM/camera/enable',
                'depth_sensor' : '/CAM/range_finder/enable',
                }


enable_request = set_intRequest()
enable_request.value = 1 # set 1 to allows the user to enable a camera.


""" Request & Response 
"""
print("Set visual sensor enable:")


for key, value in sensor_dict.items():

    rospy.wait_for_service(sensor_dict[key])


    try: 
        sensor_enable = rospy.ServiceProxy(sensor_dict[key], set_int)

        state = sensor_enable.call(enable_request)

        print("'{}' topic can be subscribed with '{}'".format(key, value)  )


    except rospy.ServiceException as e:
        print("Sevice call failed: ", e)









    