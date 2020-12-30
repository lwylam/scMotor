# Cable Robot System
## Basic components 
The graphical user interface (GUI) of the cable robot for tank cleaning consists of 5 major components. 
(1)	__Status bar__. This is the text on the top left corner, starting with the word “Status:” This shows the current section of programme. The details of the sequence are explained later in this documentation.
(2)	__Menu buttons__. This list of buttons is located under the status bar. Different functions in the specified section are listed here.
(3)	__Next/Exit button__. This button is at the middle right of the application. Usually it allows the user to move on to the next section of the programme, unless under unexpected situations then it shows “Exit” to terminate the programme.
(4)	__Textbox__ (black). This is the big black box on the lower half of the application. Most of the current status of the programme would be updated here. This is mainly for any system extra information.
(5)	__Attention for limit switches__. The radio button on the top right corner shows whether a limit switch is hit, and which one it is. This display would function throughout the whole sequence of the programme, as long as the Arduino is connected.

The programme operation follows the sequence below.

## Search network
The programme first checks if the wireless Arduino is connected. Upon success, "Arduino port opened." will be shown in the textbox, along with the serial communication parameters. This implies that the notification of the limit switches will be available on the top right corner.

However, if the Arduino port cannot be opened, the “Next” button will appear as an exit button to terminate the programme.

When it is determined that the system is ready for the next section of operation, click __once__ on the “Search” button (which is the next/exit button) to search for the motor network. Please __WAIT__, as this process may take several tens of seconds. Upon completion, the textbox shows the number of motors found in the network.

## Initialization
__Torque__ – Tighten cables with Torque mode
All motors run independently until -2.5% of the full torque is reached. This tightens the cable robot. Repeat this command multiple times to achieve the desired stiffness of end-effector of the cable robot. The check box on the side allow the motors to turn in opposite directions to loosen the cables at +1% of the full torque.

__Set as Home__ – Set the current pose as pre-defined home
An internally set home pose Is defined in the C++ programme, which is shown right next to the “Set as Home” button. This command sets all 8 internal motor counts to zero, and copy the home array into the in1 array, which stores the current position / pose of the cable robot.

__Update Pos__ – Update the in1 array with an external file
This command reads the pose saved in the “lastPos.txt”, then update the in1 array. The internal motor counts are updated accordingly. Please note that this file is updated upon safely exiting the application, and the last pose of the cable robot and internal motor counts are saved.
An alternative is to use Leica for finding the current position and rotation, update “lastPos.txt” manually and save the file, then finally run this command.

__Adjust Cable__ – Adjust the cable individually
The button itself will update the display of the internal motor count of the selected motor (#1-8). “+” and “-” increase and decrease the cable length respectively. “Home” is going to the motor counter zero, ie. homing. This can be used to restore the cable robot to home position after crash, motor to motor, as long as the motors' power is never cut off during crash.

If any commands or functions from this section is needed afterwards, please safely quit the programme and restart it. Upon clicking on the "Next" button, the application moves on to the operation section.

## Operation
Upon reaching this section of the programme, data logging would start, recording the modes used and the points commanded. Saved files can be found in the "Data Log" folder.


__Trajectory__ – Read from \"traj.csv\" file for pre-set trajectory
Define the trajectory in the form of 6 DoF pose and time in ms a line (eg. 2.197, -3.599, 0.744, 0, 0, 0, 350). Save the file before running this command.

IMPORTANT! Trajectory stop and pause buttons are on the right side of the application. If any of the limit switches is hit during the trajectory runtime, radio button will be turned on, the trajectory is aborted, equivalent to hitting the stop button on the application.

__Parameter__ – Parameterized trajectory
All the parameters are saved in the function void (vector<vector<double>>& points, double startPos[]) in the "scMotor.cpp" file, so if adjustments are needed, please safely quit the programme, save the new parameters and compile the programme __before__ running this command. To find the tankStart[3], tankX[3], tankY[3] coordinates, poke the tank with the end-effector __within__ the tank with the manual mode (refer to 'Manual' command below, read the in1[] from prompt). This sets the limits of where the end-effector would go. Define also the tankDepth, plateNum, and endEffectorWidth, especially when the end-effector is changed or replaced. Please refer to the comments in the .cpp file.  The generated trajectory is like a downward facing "E". 
 
IMPORTANT! Trajectory stop are pause are implemented. Cases are as listed in the trajectory command above.

__Manual__ - Manual control using the arrow buttons on the application
The direction of motions assume the user faces the cable robot on the platform. Note that the diagonal rise and fall has a pre-defined 55° movement. "home" button can be used to home the robot with trajectory planning. The white textbox below the arrows can be used to change the step size of the manual control.

IMPORTANT! Although the raising attention of the limit switch hit would abort all sorts of trajectory, this does not prevent manual control motions. Therefore please use the manual mode to move the robot to a safe pose after hitting a limit switch if such situation occurs.

After all the desired operations, it is recommended to home the robot in the manual control, because the internal counts will be lost when power is cut off. Then, click "Next" to move on to the next section of the programme, which is to prepare for shut down.

## Shut down preparation
Upon coming to this point of the programme, the log file is closed with the time saved. The "lastPos.txt" is saved with the in1 array and motor internal counts. Then, click on "Exit" to safely quit the application.

_Last updated on 30/12/2020_