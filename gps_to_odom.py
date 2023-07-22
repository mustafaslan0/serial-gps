#!/usr/bin/env python

import rospy
from sensor_msgs.msg import NavSatFix
import serial

def parse_gngga_data(gngga_data):
    # "$GNGGA" mesajını ayıklayın ve latitude, longitude ve altitude değerlerini döndürün
    gngga_parts = gngga_data.split(',')
    if gngga_parts[0] == "$GNGGA" and len(gngga_parts) >= 10:
        latitude = float(gngga_parts[2]) / 100.0
        longitude = float(gngga_parts[4]) / 100.0
        altitude = float(gngga_parts[9])
        return latitude, longitude, altitude
    else:
        return None, None, None

def gps_publisher():
    # Seri haberleşme ayarları
    serial_port = '/dev/ttyUSB0'  # GPS cihazının bağlı olduğu seri portu belirtin
    baud_rate = 9600             # GPS cihazının baud hızını belirtin (örn: 9600)

    # GPS verilerini okumak için seri portu açın
    try:
        ser = serial.Serial(serial_port, baud_rate, timeout=1)
        rospy.loginfo("Serial port opened successfully.")
    except serial.SerialException as e:
        rospy.logerr(f"Failed to open serial port: {str(e)}")
        return

    # ROS yayıncısını oluşturun
    rospy.init_node('gps_publisher_node', anonymous=True)
    gps_pub = rospy.Publisher('gps_data', NavSatFix, queue_size=10)

    # ROS hızını ayarlayın
    rate = rospy.Rate(1) # 1 Hz

    while not rospy.is_shutdown():
        try:
            # GPS verilerini seri porttan okuyun
            gps_raw_data = ser.readline().strip()
            rospy.loginfo(f"GPS Raw Data: {gps_raw_data}")

            # "$GNGGA" mesajını ayıklayın
            gps_raw_data = gps_raw_data.decode('ascii')
            latitude, longitude, altitude = parse_gngga_data(gps_raw_data)

            # Eğer mesaj başarılı bir şekilde ayıklandıysa, GPS verilerini yayınlayın
            if latitude is not None and longitude is not None and altitude is not None:
                # NavSatFix mesajını doldurun
                gps_data = NavSatFix()
                gps_data.latitude = latitude
                gps_data.longitude = longitude
                gps_data.altitude = altitude

                # GPS verilerini yayınlayın
                gps_pub.publish(gps_data)

        except serial.SerialException as e:
            rospy.logerr(f"Serial port error: {str(e)}")
        
        rate.sleep()

if __name__ == '__main__':
    try:
        gps_publisher()
    except rospy.ROSInterruptException:
        pass
