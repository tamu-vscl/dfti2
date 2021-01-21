#!/usr/bin/env python
import rospy
import serial
from dfti2.msg import dftiData
import serial.tools.list_ports

MOTOR = 0
SERVO = 1


def send_to_arduino(arduino,data):
    arduino.write(str(data).encode('utf-8'))
    recived = str(arduino.readline()[:-2])
    if not recived == str(data):
        raise Exception("Failed to communicate with arduino while sending {} arduino received {}".format(data,recived))


pins = rospy.get_param("arduino_pins")
stream_rate = rospy.get_param("stream_rate")

sensors = []
for pin in pins:
    sensors.append({"pin":pin,"name":"S","type":SERVO,"state":-1})


dfti_data_pub = rospy.Publisher('dfti_data',dftiData,queue_size=1000)
rospy.init_node('arduino_publisher')
msg = dftiData()

number_of_sensors = len(sensors)


ports = list(serial.tools.list_ports.comports())
port = ""
for p in ports:
    if "Arduino" in p.manufacturer:
        port = p.device
        rospy.loginfo("Using arduino on port " + port) # only works with one arduino
        break

if port == "":
    raise Exception("Failed to find Arduino.")
    
arduino = serial.Serial(port,115200)

if arduino.readline()[:-2] == "a":

    for sensor in sensors:
        send_to_arduino(arduino,sensor["type"])
        send_to_arduino(arduino,sensor["pin"])

    send_to_arduino(arduino,stream_rate)

    while not rospy.is_shutdown():
        msg.header.stamp = rospy.Time.now()
        if arduino.readline()[:-2] == "c":
            for sensor in sensors:
                sensor["state"] = float(arduino.readline()[:-2])
                if not sensor["state"] == 0:
                    msg.type = sensor["name"]
                    msg.data = sensor["state"]
                    dfti_data_pub.publish(msg)

arduino.close()
