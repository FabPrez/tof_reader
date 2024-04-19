#! /usr/bin/env python3

import rospy
from std_msgs.msg import Int32MultiArray
import serial
import struct

def read_and_publish_data():
    try:
        ser.flushInput()
        ser.write(b'1')
        value = ser.read(128)  # Leggi esattamente 128 byte
        
        # Verifica se la lunghezza di value è esattamente 128 byte
        if len(value) == 128:
            # Converti i byte in un array di interi
            interi = []
            for i in range(0, len(value), 2):
                intero = struct.unpack('<H', value[i:i+2])[0]
                interi.append(intero)
            
            # Pubblica l'array di interi come messaggio Int32MultiArray
            array_msg = Int32MultiArray(data=interi)
            pub.publish(array_msg)
        # else:
            # rospy.logwarn("La lunghezza dei dati letti non è 128 byte.")
    except PermissionError:
        rospy.logerr("Non si hanno i permessi della porta. Eseguire 'sudo chmod 666 /dev/ttyACM0' per abilitare la lettura.")

if __name__ == '__main__': 
    rospy.init_node('tof_distance_publisher')
    ser = serial.Serial('/dev/ttyACM0', 115200, 8, 'N', 1, timeout=1)
    pub = rospy.Publisher('tof_distance_data', Int32MultiArray, queue_size=10)

    rate = rospy.Rate(10)  # Frequenza di pubblicazione (10 Hz)
    
    rospy.loginfo("Inizio lettura e pubblicazione dei dati")
    while not rospy.is_shutdown():
        read_and_publish_data()
        rate.sleep()

    ser.close()

