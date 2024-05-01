#!/usr/bin/env python
import rospy
from robotnik_msgs.srv import set_digital_output, set_digital_outputRequest
from rb1_base_beacon.srv import SetBeaconLight, SetBeaconLightResponse

digital_output_green_light = 4
digital_output_red_light   = 5
GREEN_LIGHT = 1
RED_LIGHT   = -1
BOTH_LIGHTS = 2
NO_LIGHTS   = 0

def setLightSrvCallback(req):
    response = SetBeaconLightResponse()
    response.ret = True

    # Enable only green light
    if req.light == GREEN_LIGHT:  
        setDigitalOutput(digital_output_green_light, True)
        setDigitalOutput(digital_output_red_light, False)
    # Enable only red light
    elif req.light == RED_LIGHT:   
        setDigitalOutput(digital_output_green_light, False)
        setDigitalOutput(digital_output_red_light, True)
    # Enable both lights
    elif req.light == BOTH_LIGHTS:
        setDigitalOutput(digital_output_green_light, True)
        setDigitalOutput(digital_output_red_light, True)
    # Disable both lights
    elif req.light == NO_LIGHTS:                       
        setDigitalOutput(digital_output_green_light, False)
        setDigitalOutput(digital_output_red_light, False)
    else:
        response.ret  = False
           
    return response


def setDigitalOutput(number, value):
    rospy.wait_for_service('/rb1_base/robotnik_base_hw/set_digital_output')
    try:
        digital_output = rospy.ServiceProxy('/rb1_base/robotnik_base_hw/set_digital_output', set_digital_output)
        request = set_digital_outputRequest()
        request.value = value
        request.output = number
        response = digital_output(request)
    except rospy.ServiceException, e:
        print("Service call failed")

def main():
    rospy.init_node('rb1_beacon_node')
    service = rospy.Service('set_beacon_light', SetBeaconLight, setLightSrvCallback)
    rospy.spin()

if __name__ == "__main__":
    main()