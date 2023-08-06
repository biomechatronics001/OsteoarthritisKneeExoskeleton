Change log:

v0.4.2
- Modified the shape of the sit-to-stand assistance profile; assist_mode = 9; GUI mode name = Sit-to-Stand2 (IMU). See this file: /Dropbox/BiomechatronicsLab/Projects/Knee Osteoarthritis Exoskeleton/Code/Junxi/Sit-to-stand/torqueShapeTest.m

v0.4.3
- Show warning message if no SD card in Teensy
- Include date and time in the filename for data logging

v0.4.4
- Save assistance profile parameters along with logged data (XXX-Parameters.txt). 
- Add a subplot to the bottom left corner of the GUI to additionally display IMU angle vs gait phase. And also show both left and right side data on all of the 3 subplots there
- Updated torque constant related parameters for Tmotor AK80-9 actuators
