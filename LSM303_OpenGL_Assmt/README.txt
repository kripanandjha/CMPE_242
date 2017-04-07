This is the assignment 1 part B
The assignemnt is interfacing LSM303DLHC sensor to Jetson TK1
through I2C interface.

The assignment is mainly getting magnetometer sensor data from LSM303
and after getting X, Y and Z axis data, map it to an angle. that means 
based on motion of LSM303DLHC map the X and Y axis to angle which will
show EAST, WEST, NORTH and SOUTH direction. 

I have further send this angle to an OpenGL thread running in parallel
in jetson board which will in compass image show direction LSM303 is pointing
it, show run the application in jetson with OpenGL support you need to connect
external display with HDMI port in Jetson TK1.
