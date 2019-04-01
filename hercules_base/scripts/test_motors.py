import Motors
import time

LEFT_TRIM   = 0
RIGHT_TRIM  = 0

robot = Motors.Motors(left_trim=LEFT_TRIM, right_trim=RIGHT_TRIM)

# Now move the robot around!
# Each call below takes two parameters:
#  - speed: The speed of the movement, a value from 0-255.  The higher the value
#           the faster the movement.  You need to start with a value around 100
#           to get enough torque to move the robot.
#  - time (seconds):  Amount of time to perform the movement.  After moving for
#                     this amount of seconds the robot will stop.  This parameter
#                     is optional and if not specified the robot will start moving
#                     forever.
robot.right(160)      # Spin left at speed 200 for 0.5 seconds.
# Spin in place slowly for a few seconds.
#robot.right(100)  # No time is specified so the robot will start spinning forever.
time.sleep(10.0)   # Pause for a few seconds while the robot spins (you could do
                  # other processing here though!).
robot.stop()     # Stop the robot from moving.

# Now move backwards and spin right a few times.
#robot.right(200, 0.5)
