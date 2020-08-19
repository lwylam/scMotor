# scMotor

## Preparation
Download sc_motor_drive.exe along with the .dll, .csv and .txt files, save them in the same folder.
The programme can be run either from cmd or directly.

If compilation of the programme is needed, then download the entire package. VS code has been used in developing this programme.

If the programme is run correctly, it would enable all 8 motors sequentially, showing a list of the motors available. Then the menu of the sections will be shown.

## Programme menu
The programme is composed of two sections, the first is the initialization, the second is operation mode.
Just type the letter (lower case is preferred) for the selected operation, then "enter".

### Initialization
t - Tighten cables with Torque mode
- Default torque is -2.5% of the full torque of the motor, this value can be changed in the .cpp where it defines "float targetTorque = -2.5;".
- Each of the 8 motors will run at velocity mode until the desired torque is reached. Motors will run at reversed velocity if measured torque is too large.
- Recommended to type in "ttttttttttttttttttttttt" (ie. lots of "t"-s) so that all the motors will loop through the process until the cables are tightened enough.
- Run this command to tighten all cables evenly (for multiple times).
- Note that this command does not update the motor counts.
- Recommended to use the "s" command right afterwards to set current position as "home".

y - Loose the cables
- Revised version of "t" command to loosen the cables. (Further details please refer to "t" command above)
- Default releasing torque is +1%.

s - Set current position as home
- Set all 8 motors counter to zero.
- Copy home[] into in1[], the current position array.
- Display the home coordinates, pre-set in the programme (updating the home coordinates would require a compilation of the programme T^T)

h - Move to Home
- Move all motors to their counter "zero".
- __WARNING!__ This command runs the motors __without__ trajectory planning. Use this command only when the robot is near home position and to make sure all motors are at the "zero position".

8 - Manually adjust cable lengths
- This command is used to adjust a motor individually. The sub-menu is accessed by first enter the motor ID (ie. 0 to 7), then "enter".
- 'a' and 'd' for increase and decrease the cable length respectively.
- 'i' for reading the motor counter
- 'h' for going to the motor counter zero, ie. homing. This can be used to restore the cable robot to home position after crash, motor to motor, as long as the motors' power is never cut off during crash.
- 'b' for exiting "8" command, and back to the initialization menu.
- any other keys to deselect the current motor. Numbers can then be entered to select another motor.

u - Update position from external "currentPos.csv" file
- Read the "currentPos.csv" file and update the in1[] array. The motor counters will be updated accordingly.
- Please use Leica to find the current position, then update the "currentPos.csv" and save it before running this command.
- __WARNING!__ This command has never been tested on the system. It may have bug.

n - Move on to Next section
- Move on to the next section of the programme. Menu of the next section will appear.
- If any commands or functions from this section is needed afterwards, please safely quit the programme and restart it. (ie. there is no going back QAQ)

### Operation mode
t - Read from \"traj.csv\" file for pre-set trajectory
- Define the trajectory in the form of 6 DoF pose and time in ms a line (eg. 2.197, -3.599, 0.744, 0, 0, 0, 350).
- Save the file before running the "t" command.
- __IMPORTANT!__ System emergency stop is implemented. Trajectory will pause upon any keyboard keys hit. A confirmation question will appear, so you can decide whether to resume or abort the trajectory.

m - Manual input using w,a,s,d,r,f,g,v
- 'w' and 's' for + and - in y-direction movement
- 'a' and 'd' for - and + in x-direction movement
- 'r' and 'f' for + and - in z-direction movement
- 'g' and 'v' for diagonal rise and fall with pre-defined 55° movement
- 'h' for homing with trajectory planning
- 'p' for changing step size. Note that the unit is in meters.
- 'q' for exiting manual input mode.

p - Parameterized trajectory
- __WARNING!__ This piece of code has never been tested on the system, so good luck. XP
- All the parameters are saved in the "parameter_traj.cpp" file, so please safely quit the programme, save the new parameters and compile sc_motor_drive.cpp __before__ running this command.
- To find the tankStart[3], tankX[3], tankY[3] coordinates, poke the tank with the end-effector __within__ the tank with the manual mode (refer to 'm' command above, read the in1[] from prompt). This sets the limits of where the end-effector would go.
- Please also define tankDepth, plateNum, and endEffectorWidth, especially when the end-effector is changed or replaced. Please refer to the comments in the .cpp file.
- The generated trajectory is like a downward facing "E". 
- __IMPORTANT!__ System emergency stop is implemented. (Further details please refer to 't' command above)

n - Prepare to disable motors and exit programme
- Move on to the next section of the programme, which is to prepare for shut down.

## Correctly quitting the programme
Upon coming to this point of the programme, it will prompt you whether you would like to move the robot to its home before disabling the motors.
If you are to cut off the power supply afterwards, please home the robot first, because the internal counts of the motors will be lost.
Otherwise, you can simply update the currentPos.csv upon restarting the programme.
The last position of the robot will also be saved in the "data.txt" file.

If you have not safely come to this part of the programme, then the programme has probably crashed.....
But hope that you can find a way to recover the robot / system from the functions in the initialization section when you restart the programme. Good luck ;)

_Last updated on 19/8/2020_
