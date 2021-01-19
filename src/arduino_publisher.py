#!/usr/bin/env python
import rospy
import serial
from dfti2.msg import dftiData

MOTOR = 0
SERVO = 1

sensors = [{"pin":7, "name":"M","type":MOTOR,"state":-1}]
           #{"pin":59,"name":"S","type":SERVO,"state":-1}]
stream_rate = 100


dfti_data_pub = rospy.Publisher('dfti_data',dftiData,queue_size=1000)
rospy.init_node('arduino_publisher')
msg = dftiData()



number_of_sensors = len(sensors)

arduino = serial.Serial('/dev/ttyACM0',115200)
ready = arduino.readline()[:-2]

if ready == "a":
    arduino.write(str(number_of_sensors).encode('utf-8'))
    recived = int(arduino.readline()[:-2])
    if not recived == number_of_sensors:
        raise Exception("Failed to communicate with arduino while sending {} arduino received {}".format(number_of_sensors,recived))

    for sensor in sensors:
        arduino.write(str(sensor["type"]).encode('utf-8'))
        recived = int(arduino.readline()[:-2])
        if not recived == sensor["type"]:
            raise Exception("Failed to communicate with arduino while sending {} arduino received {}".format(sensor["type"],recived))

        arduino.write(str(sensor["pin"]).encode('utf-8'))
        recived = int(arduino.readline()[:-2])
        if not recived == sensor["pin"]:
            raise Exception("Failed to communicate with arduino while sending {} arduino received {}".format(sensor["pin"],recived))

        arduino.write(str(stream_rate).encode('utf-8'))

    recived = int(arduino.readline()[:-2])
    if not recived == stream_rate:
        raise Exception("Failed to communicate with arduino while sending {} arduino received {}".format(stream_rate,recived))

    ready = arduino.readline()[:-2]
    if ready == "b":
        while not rospy.is_shutdown():
            # ready = arduino.readline()[:-2]
            # if ready == "c":
            msg.header.stamp = rospy.Time.now()
            # arduino.write(str(1).encode('utf-8'))
            for sensor in sensors:
                sensor["state"] = float(arduino.readline()[:-2])
                if not sensor["state"] == 0:
                    msg.type = sensor["name"]
                    msg.data = sensor["state"]
                    dfti_data_pub.publish(msg)
            # else:
            #     print(ready)
    else:
        print(ready)
else:
    print(ready)

arduino.close()
