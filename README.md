# Commands format

The command has the following format : `<%c-%d>`.
It must start and end with "<" and ">" respectively to be accepted.
The first character indicates the function to be executed and the number indicates a value passed to it. The value can be positive or negative.
If the character is valid but the value is outside of the possible range, the command is discarded.

For example : to move the pot's fork to 75 mm from the floor we will send "<F-75>".

# Commands list

- A-x : sets angle to which the arm moves when it is deployed
   - x : value between min and max for the arm's servo
  
- D-x : disable stepper motor drivers (good to do when not in use during testing to avoid overheating the drivers/motors)
   - x : whatever

- E-x : enable stepper motor drivers (also does a homing)
   - x : whatever
  
- F-x : sets lower fork (pot's fork) height in mm
   - 0 <= x <= 142
 
- f-x : sets upper fork (plant's fork) height in mm
   - 0 <= x <= 142

- G-x : sets gripper position
   - x = 0 : gripper open (0°)
   - x = 1 : gripper closed (180°)
   - 2 <= x <= 180 : set servo angle

- S-x : sets actuators State
   - x = 0 : retracted
   - x = 1 : fork deployed
   - x = 2 : arm deployed

- T-x : sets angle to which the forks moves when they are deployed
   - x : value between min and max for the forks's servo

- W-x : sets wheel speed
   - -50 <= x < 0 : wheel turns counterclockwise
   - x = 0 : wheel is stopped
   - 0 < x <= 50 : wheel turns clockwise
