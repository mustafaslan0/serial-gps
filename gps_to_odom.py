import serial
import rospy
import argparse
from sensor_msgs.msg import NavSatFix

# Argümanları parse et
# python gps_node.py /dev/ttyUSB0

parser = argparse.ArgumentParser(description='Read GPS data from a serial port and publish it to a ROS topic.')
parser.add_argument('port', type=str, help='The name of the serial port to read from.')
args = parser.parse_args()

# Seri portu aç
ser = serial.Serial(args.port, 9600)

# ROS publisher'ı başlat
pub = rospy.Publisher('gps', NavSatFix, queue_size=10)
rospy.init_node('gps_node', anonymous=True)
rate = rospy.Rate(10)  # 10hz

while not rospy.is_shutdown():
    line = ser.readline().decode('utf-8')  # Satırı oku ve decode et
    if line.startswith('$GNGGA'):  # Eğer satır $GNGGA ile başlıyorsa
        parts = line.split(',')  # Satırı virgüllere göre parçala
        time = parts[1]
        latitude = float(parts[2][:2]) + float(parts[2][2:])/60  # Dereceye çevir
        lat_direction = parts[3]
        longitude = float(parts[4][:3]) + float(parts[4][3:])/60  # Dereceye çevir
        lon_direction = parts[5]
        fix_quality = parts[6]
        num_of_satellites = parts[7]
        hdop = parts[8]
        altitude = parts[9]
        geoid_height = parts[11]

        # ROS mesajını oluştur
        gps_msg = NavSatFix()
        gps_msg.header.stamp = rospy.Time.now()
        gps_msg.header.frame_id = "gps"
        gps_msg.latitude = latitude if lat_direction == 'N' else -latitude
        gps_msg.longitude = longitude if lon_direction == 'E' else -longitude
        gps_msg.altitude = float(altitude)

        # Mesajı yayınla
        pub.publish(gps_msg)

        rate.sleep()
