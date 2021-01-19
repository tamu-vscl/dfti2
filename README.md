
# data_importer.m
This ros node reformats data from Ardupilot into the format needed for data logging on DFTI 2.

More specifically it does rotations into Forward Right Down format and passes on one data point at a time.

By default MAVROS uses several coordinate frames:

NED - Literally north east down on the compass rose
ENU - Literally east north up on the compass rose
FLU - Baselink - Forward left
up with respect to the vehicle
FRD - Aircraft - Forward right down with respect to the vehicle

Each of these can be earth or body fixed coordinate frames.

Also there is some discussion of a ECEF (earth centered earth fixed) coordinate frame and UTM coordinates. I have not seen the code for these though.

All units are metric.

This program uses the /mavros/local_position/odom message. From this message we get:
- pose
  - ENU position
  - ENU orientation
- twist (velocities)
  - FLU linear
  - FLU angular
This program also uses the /mavros/rc/out message. From this message we get:
  - PWM channels (signal to the ESCs for the motors)

This is then converted to
- NED Position
- Phi, Theta, Psi using their normal aviation definition of a 3-2-1 rotation (roll - pitch - yaw)
- FRD linear velocities
- FRD Angular velocities
- PWM Channels


Solution to the ODOM not publishing:
Set EK2_GPS_TYPE to 0
