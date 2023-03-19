#!/usr/bin/env python3

import rospy
from arc_ms.srv import VerifyVehicleIdToken, VerifyVehicleIdTokenResponse


def verify_vehicle_id_token(request):
    
    """
    Description:    Define the service callback function to extract the vehicle ID and token from the request.
    """

    vehicle_id = request.vehicle_id
    token = request.token

    # Verify the vehicle ID against the stored database
    if vehicle_id == "1234":
        # Check if the provided token matches the one generated earlier
        if token == "5678":
            # Return a success response
            response = VerifyVehicleIdTokenResponse()
            response.success = True
            response.message = "Vehicle ID and Token verified successfully"
            rospy.loginfo("[ARC_AUTH_NODE]: Vehicle ID and Token verified successfully")
            return response
        else:
            # Return a failure response
            response = VerifyVehicleIdTokenResponse()
            response.success = False
            response.message = "Invalid Token"
            rospy.logwarn("[ARC_AUTH_NODE]: Invalid Token")
            return response
    else:
        # Return a failure response
        response = VerifyVehicleIdTokenResponse()
        response.success = False
        response.message = "Invalid Vehicle ID"
        rospy.logwarn("[ARC_AUTH_NODE]: Invalid Vehicle ID")
        return response






def main():

    """
    Description:    
    Suggestions:    `anonymous=True` while running same node multiple times at once
    """

    rospy.init_node('arc_auth')
    rospy.loginfo("[ARC_AUTH_NODE]: Arc auth node has started")

    rospy.Service("arc_auth/verify_vid_token", VerifyVehicleIdToken, verify_vehicle_id_token)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        try:
            pass
        except Exception as e:
            pass
        print("[ARC_AUTH_NODE] Shutting down the node")