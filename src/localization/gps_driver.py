#!/usr/bin/env python3
'''
Reads and parses NMEA messages from the onboard GPS to provide
location data to the rover over LCM (/gps). Subscribes to
/rtcm and passes RTCM messages to the onboard gps to
acquire an RTK fix.
'''
import serial
import asyncio
import rospy
import threading
import numpy as np
from os import getenv
# from rover_msgs import GPS, RTCM
from pyubx2 import UBXReader, UBX_PROTOCOL, RTCM3_PROTOCOL, protocol, UBXMessage
from std_msgs.msg import Header
from sensor_msgs.msg import NavSatFix
from rtcm_msgs.msg import Message


class GPS_Driver():

    def __init__(self):
        rospy.init_node('gps_driver')
        self.port = rospy.get_param("port")
        self.baud = rospy.get_param("baud")
        self.base_station_sub = rospy.Subscriber('/rtcm', Message, self.process_rtcm)
        self.gps_pub = rospy.Publisher('/gps', NavSatFix, queue_size=1)
        self.lock = threading.Lock()


    def connect(self):
        #open connection to rover GPS
        self.ser = serial.Serial(self.port, self.baud)
        self.reader = UBXReader(self.ser, protfilter=(UBX_PROTOCOL | RTCM3_PROTOCOL))

        return self

    def exit(self) -> None:
        #close connection
        self.ser.close()

    def process_rtcm(self, data) -> None:
        print("processing RTCM")
        with self.lock:
            # rtcm_data = RTCM.decode(data)
            self.ser.write(data.message)

    def parse_rover_gps_data(self, rover_gps_data) -> None:
        msg = rover_gps_data
        # skip if message could not be parsed
        if not msg:
            return
                        
        if rover_gps_data.identity == "RXM-RTCM":
            print("RXM")
            msg_used = msg.msgUsed

            if msg_used == 0:
                print("RTCM Usage unknown\n")
            elif msg_used == 1:
                print("RTCM message not used\n")
            elif msg_used == 2:
                print("RTCM message successfully used by receiver\n")



            # rospy.loginfo(vars(rover_gps_data))
        
        if rover_gps_data.identity == "NAV-PVT":
            print("PVT")
            parsed_latitude = msg.lat
            parsed_longitude = msg.lon
            parsed_altitude = msg.hMSL
            message_header = Header(stamp=rospy.Time.now(), frame_id="base_link")

            self.gps_pub.publish(
                NavSatFix(header=message_header, latitude=parsed_latitude, longitude=parsed_longitude, altitude=parsed_altitude)
            )

            if msg.difSoln == 1:
                print("Differemtial Corrections Applied")
            

            # publidh to navsatstatus in navsatfix
            if msg.carrSoln == 0:
                print("No RTK\n")
            elif msg.carrSoln == 1:
                print("Floating RTK Fix\n")
            elif msg.carrSoln == 2:
                print("RTK FIX\n")    

        if rover_gps_data.identity == "NAV-STATUS":
            pass



    def gps_data_thread(self) -> None:
        #TODO: add more message checks if needed
        while not rospy.is_shutdown():
            with self.lock:
                if self.ser.in_waiting:
                    raw, rover_gps_data = self.reader.read()
                    parsed_gps_data = self.parse_rover_gps_data(rover_gps_data)


def main():
    #change values
    rtk_manager = GPS_Driver()
    rtk_manager.connect()
    #rover_gps_thread = threading.Thread(rtk_manager.gps_data_thread)
    #rover_gps_thread.start()
    rtk_manager.gps_data_thread()
    rtk_manager.exit()


if __name__ == "__main__":
    main()