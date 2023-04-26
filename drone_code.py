#!/usr/bin/env python
import rospy
from std_msgs.msg import String
# import split

# pymavlink 
from pymavlink import mavutil
import time
import zebu_pymavlink_util as zebu_util

connection_string = 'udpin:localhost:14551'
vehicle, res = zebu_util.connect_drone(connection_string, check = False)

# Mode Guided
vehicle.mav.command_long_send(vehicle.target_system,vehicle.target_component,mavutil.mavlink.MAV_CMD_DO_SET_MODE,0, 1, 4,0,0, 0,0,0)
msg = vehicle.recv_match(type='COMMAND_ACK', blocking=True)  

def receive_and_print_message(_topics):
    for i in _topics:
        print("Waiting for message "+str(i))
        msg = vehicle.recv_match(type=i, blocking=True,timeout = 5)
        if(type(msg) == type(None)):
            print("No message till timeout")
        else:
            print(msg)

def listen_test(_vehicle):
    msg = _vehicle.recv_match(type='LOCAL_POSITION_NED',blocking=True)
    # msg = vehicle.recv_match(type='ALTITUDE',blocking=True)
    print(msg)

def takeoff_n_land_test(connection_string, baud=56700):
    # Connecting to drone
    
    vehicle, res = zebu_util.connect_drone(connection_string, check = False)

    if(res == -1):
        return(-1)
    listen_test(vehicle)
    # Checking heartbeat
    print("Waiting for heartbeat!")
    vehicle.wait_heartbeat()
    print("Heartbeat from system %u component %u)"%(vehicle.target_system,vehicle.target_component))

    # Switching to guided mode
    zebu_util.activate_guided(vehicle)

    # Arming
    print("Sending Arm command:")
    res = zebu_util.send_long_command_till_ack(vehicle,[vehicle.target_system,vehicle.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,0,1,0,0,0,0,0,0])
    if(res==0):
        print("Armed successfully!!")
    else:
        print("Arming Failed!!!")

    print("Sending Taking off ...")
    zebu_util.activate_takeoff(vehicle,10,verbose=0)
    start_time = time.time()
    while(True):
        
        msg = vehicle.recv_match(type='NAV_CONTROLLER_OUTPUT', blocking=True,timeout=1)
        print(msg)
        curr_time = time.time()
        if(curr_time-start_time>5):
            break
    zebu_util.activate_land(vehicle)
    return(1)

def callback(data):
    as_int = (data.data)


    res = [int(x) for x in as_int.split(",")]
    x = res[0]/500
    y = res[1]/500
    print(res[0],res[1])


    # vehicle.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message
    #                  (10, vehicle.target_system, vehicle.target_component, 
    #                   mavutil.mavlink.MAV_FRAME_LOCAL_NED, int(0b110111111000), # position 0b110111111000 # velocity 0b110111000111
    #                   0,    # X Position in NED frame
    #                   0,    # Y Position in NED frame
    #                   -5,    # Z Position in NED frame (note, altitude is negative in NED)
    #                   0,    # X velocity in NED frame
    #                   0,    # Y velocity in NED frame
    #                   0,  # Z velocity in NED frame
    #                   0,    # X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
    #                   0,    # Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
    #                   0,    # Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
    #                   0,   # yaw setpoint
    #                   0))   # yaw rate setpoint

    vehicle.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message
                     (10, vehicle.target_system, vehicle.target_component, 
                      mavutil.mavlink.MAV_FRAME_LOCAL_NED, int(0b100111000111), # position 0b110111111000 # velocity 0b110111000111
                      0,    # X Position in NED frame
                      0,    # Y Position in NED frame
                      -3,    # Z Position in NED frame (note, altitude is negative in NED)
                      y,    # X velocity in NED frame
                      x,    # Y velocity in NED frame
                      z,  # Z velocity in NED frame
                      0,    # X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
                      0,    # Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
                      0,    # Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
                      0,   # yaw setpoint
                      0))   # yaw rate setpoint


    # # input check
    # res = takeoff_n_land_test(connection_string)
    # receive_and_print_message(topics)

    
    return (res)
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    value = rospy.Subscriber("chatter", String, callback)
    # print(value)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    return()

if __name__ == '__main__':
    listener()