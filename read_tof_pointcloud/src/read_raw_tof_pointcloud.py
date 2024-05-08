#! /usr/bin/env python3

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
BYTE_PER_READ = TOF_ZONE_NUMBER * POINTS_PER_ZONE * BYTE_PER_POINT
BAUD_RATE = 115200
TIMEOUT = 1

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
        # rospy.loginfo("Valori dei byte ricevuti:-----------")
        # for byte in value:
        #     rospy.loginfo(format(byte, '08b'))
        # rospy.loginfo("Valori dei byte terminati-----------")
        
        interi = []
        for i in range(0, len(value), BYTE_PER_POINT):  # Leggi i byte in coppia
            intero = combine_bytes(value[i+1], value[i])
            
            if is_negative(intero):
                intero = struct.unpack('<h', struct.pack('<H', intero))[0]
            
            interi.append(intero)
        
        # Publish the array of integers as Int32MultiArray
        array_msg = Int32MultiArray(data=interi)
        pub.publish(array_msg)

if __name__ == '__main__': 
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

# def combine_bytes(msb, lsb):
#     # Combina i due byte per formare un intero a 16 bit
#     combined_value = (msb << 8) | lsb
#     return combined_value

# def is_negative(value):
#     # Controlla il bit più significativo
#     if value & 0x8000:  # Se il bit più significativo è impostato
#         return True    # Il numero è negativo
#     else:
#         return False   # Il numero è positivo o zero

# def read_tof_pointcloud():

#     ser.flushInput()
#     ser.write(b'1')
#     value = ser.read(128*3)  
#     # Leggi esattamente 128*3 byte, ogni numero è codificato su 2 byte (quindi 64*2 = 128)
#     # e ci sono 3 numeri (distanza x, y, z) 
    
#     # Verifica se la lunghezza di value è esattamente 128*3 byte
#     if len(value) == 128*3:
#         # rospy.loginfo("Valori dei byte ricevuti:-----------")
#         # for byte in value:
#         #     rospy.loginfo(format(byte, '08b'))
#         # rospy.loginfo("Valori dei byte terminati-----------")
        
#         # Converti i byte in un array di interi
#         interi = []
#         for i in range(0, len(value), 2):  # Leggi i byte in coppia
#             # Inverti l'ordine dei byte e combina i byte in un intero a 16 bit
#             intero = combine_bytes(value[i+1], value[i])
            
#             # Controlla se il numero è negativo
#             if is_negative(intero):
#                 # Se è negativo, converte il numero negativo a un intero a 32 bit (complemento a due)
#                 # (per mantenere il segno corretto)
#                 intero = struct.unpack('<h', struct.pack('<H', intero))[0]
            
#             interi.append(intero)
        
#         # Pubblica l'array di interi come messaggio Int32MultiArray
#         array_msg = Int32MultiArray(data=interi)
#         pub.publish(array_msg)

# if __name__ == '__main__': 
#     rospy.init_node('tof_distance_publisher')
#     try:
#         ser = serial.Serial('/dev/ttyACM0', 115200, 8, 'N', 1, timeout=1)
#     except serial.SerialException as e:
#         rospy.logerr("Non si hanno i permessi della porta. Eseguire 'sudo chmod 666 /dev/ttyACM0' per abilitare la lettura.")
#         exit()
        
#     pub = rospy.Publisher('tof_pointcloud_data', Int32MultiArray, queue_size=10)

#     rate = rospy.Rate(10)  # Frequenza di pubblicazione (10 Hz)
    
#     rospy.loginfo("Inizio lettura e pubblicazione dei dati")
#     while not rospy.is_shutdown():
#         read_tof_pointcloud()
#         rate.sleep()

#     ser.close()

