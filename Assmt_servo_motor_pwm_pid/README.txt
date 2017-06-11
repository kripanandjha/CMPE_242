The assignment is about controlling the servo motor on LPCNOD1758 side.
whic will run through pwm single. Using LSM303 NOD calucalates the angle rotated,
to ARMSVR side. Server will calculate the pid error and finds new pwm single
and direction of motion and send it back to the LPCNOD side. LPCNOD will change
the motor speed and direction based on input receive and after 30 ms it 
calculates new angle and send it back to the server. Based on PID error 
calculation motor will keep rotating untill pid error reaches zero.
