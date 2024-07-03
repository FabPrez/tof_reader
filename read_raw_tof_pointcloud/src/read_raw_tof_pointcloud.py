#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32MultiArray
import serial
import struct

# Constants and configuration
SERIAL_PORT = '/dev/ttyACM0'
TYPE_BIT_SENT_BY_TOF = 16 # TOF IS SENDING 16 BIT PER DATA: 2 BYTES
TOF_ZONE_NUMBER = 64
POINTS_PER_ZONE = 3 # X, Y, Z COORDINATES
BYTE_PER_POINT = 2 # TOF IS USING 2 BYTES PER POINT
BAUD_RATE = 115200
TIMEOUT = 1

def combine_bytes(msb, lsb):
    combined_value = (msb << int(TYPE_BIT_SENT_BY_TOF/BYTE_PER_POINT)) | lsb   
    return combined_value

def is_negative(value):
    if value & 0x8000: 
        return True    # The number is negative
    else:
        return False   # The number is positive or zero

def read_tof_pointcloud(ser, publishers):
    try:
        ser.flushInput()
        ser.write(b'1')
        value = ser.read(BYTE_PER_READ)  

        if len(value) == BYTE_PER_READ:
            interi = []
            for i in range(0, len(value), BYTE_PER_POINT):  # Read the bytes in pairs
                intero = combine_bytes(value[i+1], value[i])
                if is_negative(intero):
                    intero = struct.unpack('<h', struct.pack('<H', intero))[0]
                interi.append(intero)
            
            if number_of_tofs > 1:
                array_to_publish = interi[:-1]
                tof_number = interi[-1]
                array_msg = Int32MultiArray(data=array_to_publish)
                publishers[tof_number].publish(array_msg)
            else:
                array_msg = Int32MultiArray(data=interi)
                publishers[0].publish(array_msg)

        else:
            rospy.logwarn("Received incomplete data from TOF sensor")
    except Exception as e:
        rospy.logerr(f"Error reading TOF pointcloud: {e}")

if __name__ == '__main__': 
    rospy.init_node('TOF_POINTCLOUD_READER')
    number_of_tofs = rospy.get_param('~number_of_tofs', 1)
    rospy.loginfo(f"Starting TOF READER for {number_of_tofs} TOFs ")
    publishers = {}
    if number_of_tofs > 1:
        BYTE_PER_READ = TOF_ZONE_NUMBER * POINTS_PER_ZONE * BYTE_PER_POINT + 2 # 2 BYTES FOR THE TOF NUMBER
    else:
        BYTE_PER_READ = TOF_ZONE_NUMBER * POINTS_PER_ZONE * BYTE_PER_POINT
        
    for i in range(number_of_tofs):
        topic_name = f'tof_raw_pointcloud_data{i+1}'
        rospy.loginfo(f"TOFs n°{i+1} publis on topic {topic_name}")
        publishers[i] = rospy.Publisher(topic_name, Int32MultiArray, queue_size=10)
    
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=TIMEOUT)
    except serial.SerialException as e:
        rospy.logerr(f"Failed to open serial port: {e}, run 'sudo chmod 666 /dev/ttyACM0' ")
        exit()
        
    rate = rospy.Rate(10) 
    
    while not rospy.is_shutdown():
        read_tof_pointcloud(ser, publishers)
        rate.sleep()
    ser.close()
