#! /usr/bin/env python3

import rospy
from std_msgs.msg import Int32MultiArray
import serial
import struct

# Constants and configuration
SERIAL_PORT = '/dev/ttyACM0'
TOF_NUMBER = 0
TYPE_BIT_SENT_BY_TOF = 16 # TOF IS SENDING 16 BIT PER DATA: 2 BYTES
TOF_ZONE_NUMBER = 64
POINTS_PER_ZONE = 3 # X, Y, Z COORDINATES
BYTE_PER_POINT = 2 # TOF IS USING 2 BYTES PER POINT
BAUD_RATE = 115200
TIMEOUT = 1
BYTE_PER_READ = TOF_ZONE_NUMBER * POINTS_PER_ZONE * BYTE_PER_POINT + (TOF_NUMBER+1)*2 # 2 BYTES FOR THE TOF NUMBER

def combine_bytes(msb, lsb):
    # COMBINE THE TWO BYTES TO FORM A 16-BIT INTEGER
    combined_value = (msb << int(TYPE_BIT_SENT_BY_TOF/BYTE_PER_POINT)) | lsb   
    return combined_value

def is_negative(value):
    # CHECK THE MOST SIGNIFICANT BIT, LOOKING FOR NEGATIVE NUMBERS (2 COMPLEMENT)
    if value & 0x8000: 
        return True    # The number is negative
    else:
        return False   # The number is positive or zero

def read_tof_pointcloud():
    ser.flushInput()
    ser.write(b'1')
    value = ser.read(BYTE_PER_READ)  

    # Verifica se la lunghezza di value è esattamente BYTE_PER_READ byte
    if len(value) == BYTE_PER_READ:
        # JUST FOR DEBUGGING --------------------------------------
        rospy.loginfo("Valori dei byte ricevuti:-----------")
        for byte in value:
            rospy.loginfo(format(byte, '08b'))
        rospy.loginfo("Valori dei byte terminati-----------")
        
        interi = []
        for i in range(0, len(value), BYTE_PER_POINT):  # Leggi i byte in coppia
            intero = combine_bytes(value[i+1], value[i])
            
            if is_negative(intero):
                intero = struct.unpack('<h', struct.pack('<H', intero))[0]
            
            interi.append(intero)
            
        array_to_publish = interi[:-1]
        # array_to_publish = interi
        # Publish the array of integers as Int32MultiArray
        array_msg = Int32MultiArray(data=array_to_publish)
        
        rospy.loginfo("Valori ultimo intero---")
        rospy.loginfo(interi[-1])
        rospy.loginfo("-----------")
            
        if interi[-1] == TOF_NUMBER:
            rospy.loginfo(f"TOF NUMBER: {TOF_NUMBER}, Is Publishing!")
            pub.publish(array_msg)

if __name__ == '__main__': 
    # TOF_NUMBER = rospy.get_param('/tof_to_pointcloud/tof_number')
    # rospy.loginfo(f"Il valore di my_param è: {TOF_NUMBER}")
    
    rospy.init_node('TOF_POINTCLODUD_READER')
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, 8, 'N', 1, timeout=TIMEOUT)
    except serial.SerialException as e:
        rospy.logerr(f"Permission denied. Run 'sudo chmod 666 {SERIAL_PORT}' to enable reading.")
        exit()
        
    pub = rospy.Publisher('tof_raw_pointcloud_data', Int32MultiArray, queue_size=10)
    rate = rospy.Rate(10) 
    
    rospy.loginfo("Start reading and publishing data")
    
    
    while not rospy.is_shutdown():
        read_tof_pointcloud()
        rate.sleep()
    ser.close()

