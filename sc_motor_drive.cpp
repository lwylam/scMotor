#include <vector>
#include <string>
#include <fstream>
#include <iostream>
#include <sstream>
#include <algorithm>
#include <chrono>
#include <ctime>
#include <thread>
#include <stdio.h>
#include <conio.h>
#include <windows.h>
#include "pose_to_length.h"
#include "parameter_traj.h"
#include "Dependencies\sFoundation20\inc\pubSysCls.h"

using namespace std;
using namespace sFnd;

int CheckMotorNetwork();
void RunTrajPoints();
void SendMotorCmd(int n);
void SendMotorTrq(int n);
void SolveCubicCoef(int loop_i);
void SendMotorGrp(bool IsTorque = false);
int SolveParaBlend(int loop_i, bool showAttention = false);
int32_t ToMotorCmd(int motorID, double length);
void TrjHome();
bool CheckLimits();

vector<string> comHubPorts;
vector<INode*> nodeList; // create a list for each node
vector<vector<double>> points;
vector<double> spcLimit;
unsigned int portCount;
const int nodeNum = 8; // !!!!! IMPORTANT !!!!! Put in the number of motors before compiling the programme
double step = 0.01; // in meters, for manual control
float targetTorque = -2.5; // in %, -ve for tension, also need to UPDATE in switch case 't'!!!!!!!!!
const int MILLIS_TO_NEXT_FRAME = 35; // note the basic calculation time is abt 16ms
double home[6] = {2.216, -3.582, 1.035, 0, 0, 0}; // home posisiton
double offset[8]; // L0, from "zero posi1tion", will be updated by "set home" command
double in1[6] = {2.211, -3.482, 1.012, 0, 0, 0};
double out1[8] = {2.87451, 2.59438, 2.70184, 2.40053, 2.46908, 2.15523, 2.65123, 2.35983}; // assume there are 8 motors
double a[6], b[6], c[6], d[6], e[6], f[6], g[6], tb[6]; // trajectory coefficients
ofstream logfile;

int main()
{
    SysManager* myMgr = SysManager::Instance();
    
    // Start the programme, scan motors in network
    try{
        if (CheckMotorNetwork() < 0){
            cout << "Motor network not available. Exit programme." << endl;
            return -1;
        }
    }
    catch(sFnd::mnErr& theErr) {    //This catch statement will intercept any error from the Class library
        printf("Port Failed to open, Check to ensure correct Port number and that ClearView is not using the Port\n");  
        printf("Caught error: addr=%d, err=0x%08x\nmsg=%s\n", theErr.TheAddr, theErr.ErrorCode, theErr.ErrorMsg);
        return -1;
    }
    IPort &myPort = myMgr->Ports(0);

    cout << "Motor network available. Pick from menu for the next action:\nt - Tighten cables with Torque mode\ny - Loose the cables\ns - Set current position as home\nh - Move to Home\n8 - Manually adjust cable lengths\nu - Update current position from external file\ni - Info: show menu\nn - Move on to Next step" << endl;
    pose_to_length(home, offset); // save offset values according to home pose
    char cmd;
    try{
        do {
            bool allDone = false, stabilized = false;
            cin >> cmd;
            switch (cmd){
                case 'i':   // Show menu
                    cout << "Pick from menu for the next action:\nt - Tighten cables with Torque mode\ny - Loose the cables\ns - Set current position as home\nh - Move to Home\n8 - Manually adjust cable lengths\nu - Update current position from external file\ni - Info: show menu\nn - Move on to Next step" << endl;
                    break;
                case 'y':   // Loosen cables using positive torque
                    targetTorque = 1;
                case 't':   // Tighten cables according to torque
                    cout << "Current target torque = " << targetTorque << endl;
                    for(INode* n : nodeList){ n->Motion.AccLimit = 200; }
                    while(!stabilized) {
                        SendMotorGrp(true);
                        Sleep(50);
                        for (int n = 0; n < nodeList.size(); n++){
                            if(nodeList[n]->Motion.TrqCommanded.Value() > targetTorque) { break; }
                            stabilized = true;
                        }
                    } 
                    for(INode* n : nodeList){
                        n->Motion.TrqCommanded.Refresh();
                        cout << n->Motion.TrqCommanded.Value() << "\t";
                        n->Motion.MoveVelStart(0);
                        n->Motion.AccLimit = 40000;
                    }
                    cout << "\nTorque mode tightening completed" << endl;
                    targetTorque = -2.5;
                    break;
                case 's':   // Set zero
                    for (int n = 0; n < nodeList.size(); n++){
                        nodeList[n]->Motion.AddToPosition(-nodeList[n]->Motion.PosnMeasured.Value()); // Zeroing the number space around the current Measured Position
                    }
                    copy(begin(home), end(home), begin(in1)); // copy home array into input array
                    cout << "Setting zero completed" << endl;
                    cout <<  "Home coordinates: " << in1[0] << ", " << in1[1] << ", " << in1[2] << ", " << in1[3] << ", " << in1[4] << ", " << in1[5] << endl;
                    break;
                case 'h':   // Homing
                    allDone = false;
                    for (int n = 0; n<myPort.NodeCount(); n++) { 
                        nodeList[n]->Motion.MoveWentDone();
                        nodeList[n]->Motion.MovePosnStart(0, true); // absolute position
                    }
                    while(!allDone) {
                        for (INode* n : nodeList) {
                            if(!n->Motion.MoveIsDone()) { break; }
                            allDone = true;
                        }
                    }
                    cout << "Homing completed" << endl;
                    break;
                case '8':   // Manual cable adjustment
                    cout << "0 to 7 - motor id to adjust cable length\na or d - increase or decrease cable length\nb - Back to previous menu\n";
                    while(cmd != 'b'){
                        cin >> cmd;
                        if('/' < cmd && cmd < nodeList.size()+48){
                            int id = cmd - 48;
                            int sCount = ToMotorCmd(-1, step) / 5;
                            cout << "Motor "<< cmd <<" selected.\n";
                            do{
                                cmd = getch();
                                switch(cmd){
                                    case 'a':
                                        nodeList[id]->Motion.MovePosnStart(sCount);
                                        break;
                                    case 'd':
                                        nodeList[id]->Motion.MovePosnStart(-sCount);
                                        break;
                                    case 'i':
                                        nodeList[id]->Motion.PosnMeasured.Refresh();
                                        cout << (double) nodeList[id]->Motion.PosnMeasured << endl;
                                        break;
                                    case 'h':
                                        nodeList[id]->Motion.VelLimit = 300;
                                        nodeList[id]->Motion.MoveWentDone();
                                        nodeList[id]->Motion.MovePosnStart(0, true);
                                        while(!nodeList[id]->Motion.MoveIsDone()){}
                                        cout << "Individual homing completed.\n";
                                        nodeList[id]->Motion.VelLimit = 3000;
                                        break;
                                }                        
                                Sleep(100); // do we need this?
                            }while(cmd =='a'|| cmd =='d' || cmd =='h' || cmd =='i');
                            cout << "Motor "<< id <<" deselected.\n";
                        }
                    }
                    cout << "Manual adjustment terminated" << endl;
                    break;
                case 'u':   // Update in1[] and offset[] from csv file
                    ifstream file ("currentPos.csv");//ifstream file ("home.csv"); //
                    string temp;
                    int count = 0;
                    if(file.is_open()){
                        try{
                            while (file >> temp){
                                // home[count++] = stod(temp); // convert string to double stod()
                                in1[count++] = stod(temp); // convert string to double stod()
                            }
                            cout << "Completed updating from external Current Pose file" << endl; //"Completed updating from external pose file"
                        }
                        catch(int e){ cout << "Check if home.csv matches the home input no." << endl; }
                        // pose_to_length(home, offset); // save offset values according to home pose
                        // copy(begin(home), end(home), begin(in1)); // copy home array into input array
                        
                        // Update each motor cout according to current in1[]
                        pose_to_length(in1, out1);
                        for (int n = 0; n < nodeList.size(); n++){
                            int32_t step = ToMotorCmd(n, out1[n]);
                            nodeList[n]->Motion.PosnMeasured.Refresh();
                            nodeList[n]->Motion.AddToPosition(-nodeList[n]->Motion.PosnMeasured.Value() + step);
                        }
                        cout << "Updating motor counts completed" << endl;
                        cout << "Current coordinates: " << in1[0] << ", " << in1[1] << ", " << in1[2] << ", " << in1[3] << ", " << in1[4] << ", " << in1[5] << endl;
                        cout << "Motor internal counts: ";
                        for (int id = 0; id < 8; id++){
                            nodeList[id]->Motion.PosnMeasured.Refresh();
                            cout << (double) nodeList[id]->Motion.PosnMeasured << "\t";
                        }
                        cout << endl;
                    }
                    break;
                
            }
        } while(cmd != 'n');
    }
    catch(sFnd::mnErr& theErr) {    //This catch statement will intercept any error from the Class library
        printf("ERROR: Motor command failed.\n");  
        printf("Caught error: addr=%d, err=0x%08x\nmsg=%s\n", theErr.TheAddr, theErr.ErrorCode, theErr.ErrorMsg);
        return -3;
    }
    
    time_t now = time(0);
    tm *fn = localtime(&now);
    string fName = to_string(1900+fn->tm_year) +"-"+ to_string(1+fn->tm_mon) +"-"+ to_string(fn->tm_mday) +" "+ to_string(fn->tm_hour) +"."+ to_string(fn->tm_min);
    logfile.open ("Data Log\\" +fName+ ".txt");
    if(logfile.is_open()){ cout << "--------------Data logging has started--------------\n"; }
    logfile << fName << endl;

    cout << "Choose from menu for cable robot motion:\nt - Read from \"traj.csv\" file for pre-set trajectory\nm - Manual input using w,a,s,d,r,f,g,v\np - Parameterized trajectory\ni - Info: show menu\nn - Prepare to disable motors and exit programme" << endl;
    do {
        cin >> cmd;
        ifstream file ("traj.csv");
        vector<double> row;
        string line, word, temp;
        switch (cmd){
            case 'i':    // Show menu
                cout << "Choose from menu for cable robot motion:\nt - Read from \"traj.csv\" file for pre-set trajectory\nm - Manual input using w,a,s,d,r,f,g,v\np - Parameterized trajectory\ni - Info: show menu\nn - Prepare to disable motors and exit programme" << endl;
                break;
            case 't':   // Read traj file
            case 'T':
                // Read input file for traj-gen
                points.clear();
                if(file.is_open()){
                    while (getline(file, line)){
                        row.clear();
                        stringstream s(line);
                        while (s >> word){
                            row.push_back(stod(word)); // convert string to double stod()
                        }
                        points.push_back(row);
                    }
                    cout << "Completed reading external input file" << endl;
                }
                else{
                    cout << "Failed to read input file. Exit programme." << endl;
                    return -1;
                }
                logfile << "\n--Trajectory Mode--\n";
                RunTrajPoints();
                break;
            case 'm':   // Manual wasdrf
            case 'M':
                cout << "Press 'q' to quit manual input anytime.\n'h' for Homing.\n'x' to adjust increment step size.\n";
                logfile << "\n--Manual Mode--\n";
                while(cmd != 'q' && cmd != 'Q'){
                    cmd = getch();
                    switch(cmd){
                        case 'W':
                        case 'w':
                            in1[1] += step;
                            break;
                        case 'S':
                        case 's':
                            in1[1] -= step;
                            break;
                        case 'A':
                        case 'a':
                            in1[0] -= step;
                            break;
                        case 'D':
                        case 'd':
                            in1[0] += step;
                            break;
                        case 'R':
                        case 'r':
                            in1[2] += step;
                            break;
                        case 'F':
                        case 'f':
                            in1[2] -= step;
                            break;
                        case 'G':
                        case 'g':
                            in1[1] -= step*0.7;
                            in1[2] += step;
                            break;
                        case 'V':
                        case 'v':                        
                            in1[1] += step*0.7;
                            in1[2] -= step;
                            break;
                        case 'H':
                        case 'h':
                            cout << "Homing...\n"; 
                            TrjHome();
                            //copy(begin(home), end(home), begin(in1));
                            break;
                        case 'X':
                        case 'x':
                            cout << "Current step size: " << step << "m. Please enter new step size: ";
                            cin >> step;
                            if (!cin.good()){ cout << "Invalid input!"; }
                            else if (abs(step) > 0.1){ cout << "Warning! Step size is too large, may cause vigorious motions.";}
                            else { cout << "New step size: " << step << endl; break; }
                            cout << " Step size is now set to 0.01m.\n";
                            step = 0.01;
                            break;
                    }
                    cout << "IN: "<< in1[0] << " " << in1[1] << " " << in1[2] << " " << in1[3] << " " << in1[4] << " " << in1[5] << endl;
                    if(CheckLimits()){
                        pose_to_length(in1, out1);
                        cout << "OUT: "<<  out1[0] << " " << out1[1] << " " << out1[2] << " " << out1[3] << " " <<  out1[4] << " " << out1[5] << " " << out1[6] << " " << out1[7] << endl;
                        
                        SendMotorGrp();
                        
                        Sleep(step*1000);
                    }
                    else{
                        cout << "WARNING: Intended position out of bound!\n";
                        switch(cmd){
                            case 'W':
                            case 'w':
                                in1[1] -= step;
                                break;
                            case 'S':
                            case 's':
                                in1[1] += step;
                                break;
                            case 'A':
                            case 'a':
                                in1[0] += step;
                                break;
                            case 'D':
                            case 'd':
                                in1[0] -= step;
                                break;
                            case 'R':
                            case 'r':
                                in1[2] -= step;
                                break;
                            case 'F':
                            case 'f':
                                in1[2] += step;
                                break;
                            case 'G':
                            case 'g':
                                in1[1] += step*0.7;
                                in1[2] -= step;
                                break;
                            case 'V':
                            case 'v':                        
                                in1[1] -= step*0.7;
                                in1[2] += step;
                                break;
                        }
                    }
                }
                cout << "Quit manual control\n";
                break;
            case 'p':   // Parameterized traj
            case 'P':
                parameter_traj(points, in1);
                
                ofstream pfile;
                pfile.open ("paramTraj.txt");
                for(vector<double> line:points){
                    for(double txt:line){ pfile << txt << "\t"; }
                    pfile << endl;
                }
                pfile.close();
                
                cout << "Are you sure to run the parameterized trajectory? (y - confirm run trajectory / b - back to section menu)\n";
                cin >> cmd;
                if(cmd == 'y' || cmd == 'Y'){
                    logfile << "\n--Parameterized Trajectory Mode--\n";
                    RunTrajPoints();
                }
                break;
        }
    } while(cmd != 'n'); 
    
    Sleep(500); // wait a little more before disabling the nodes
    now = time(0);
    fn = localtime(&now);
    logfile << "\nStart shut down process at " << fn->tm_hour << ":" << fn->tm_min << endl;

    // Saving last position before quiting programme
    cout << "Saving last position...\n";
    ofstream myfile;
    myfile.open ("lastPos.txt");
    myfile << in1[0] << ", " << in1[1] << ", " << in1[2] << ", " << in1[3] << ", " << in1[4] << ", " << in1[5] << endl;
    myfile << out1[0] << ", " << out1[1] << ", " << out1[2] << ", " << out1[3] <<  ", " << out1[4] << ", " << out1[5] << ", " << out1[6] << ", " << out1[7] <<  endl;
    myfile.close();

    // Homing before shut down?
    cout << "Do you want to home the robot before shutting down? (Y/N)\n";
    cin >> cmd;
    if(cmd == 'y' || cmd == 'Y'){
        TrjHome();
        cout << "Homing completed.\n";
    }

    cout << "Disabling motors and closing ports" << endl;
    for(int i = 0; i < nodeList.size(); i++){ //Disable Nodes
        myPort.Nodes(i).EnableReq(false);
    }
    nodeList.clear();
    myMgr->PortsClose(); // Close down the ports
    logfile << "System shut down successful.\n";
    logfile.close(); // Close the log file
    
    cout << "\n" << "Done at " << myMgr->TimeStampMsec() << endl;
    return 0;   
}

int CheckMotorNetwork() {
    SysManager* myMgr = SysManager::Instance();

    sFnd::SysManager::FindComHubPorts(comHubPorts);

    cout << "Found " <<comHubPorts.size() << " SC Hubs\n";
    for (portCount = 0; portCount < comHubPorts.size(); portCount++) {
        myMgr->ComHubPort(portCount, comHubPorts[portCount].c_str());
    }
    if (portCount < 0) {
        cout << "Unable to locate SC hub port\n";
        return -1;
    }
    if(portCount==0) { return -1; } // do we need this?
    
    myMgr->PortsOpen(portCount);
    for (int i = 0; i < portCount; i++) { // check no. of nodes in each ports
        IPort &myPort = myMgr->Ports(i);
        myPort.BrakeControl.BrakeSetting(0, BRAKE_AUTOCONTROL); // do we need this?
        myPort.BrakeControl.BrakeSetting(1, BRAKE_AUTOCONTROL);
        printf(" Port[%d]: state=%d, nodes=%d\n", myPort.NetNumber(), myPort.OpenState(), myPort.NodeCount());
    
        for (int iNode = 0; iNode < myPort.NodeCount(); iNode++) {
            INode &theNode = myPort.Nodes(iNode);
            theNode.EnableReq(false); //Ensure Node is disabled before loading config file
            myMgr->Delay(200);

            printf("   Node[%d]: type=%d\n", int(iNode), theNode.Info.NodeType());
            printf("            userID: %s\n", theNode.Info.UserID.Value());
            printf("        FW version: %s\n", theNode.Info.FirmwareVersion.Value());
            printf("          Serial #: %d\n", theNode.Info.SerialNumber.Value());
            printf("             Model: %s\n", theNode.Info.Model.Value());

            theNode.Status.AlertsClear();               //Clear Alerts on node 
            theNode.Motion.NodeStopClear();             //Clear Nodestops on Node               
            theNode.EnableReq(true);                    //Enable node 
            theNode.Motion.PosnMeasured.AutoRefresh(true);
            theNode.Motion.TrqMeasured.AutoRefresh(true);
            printf("Node %d enabled. ", iNode);

            theNode.AccUnit(INode::RPM_PER_SEC);        //Set the units for Acceleration to RPM/SEC
            theNode.VelUnit(INode::RPM);                //Set the units for Velocity to RPM
            theNode.Motion.AccLimit = 40000;           //100000 Set Acceleration Limit (RPM/Sec)
            theNode.Motion.NodeStopDecelLim = 5000;
            theNode.Motion.VelLimit = 3000;             //700 Set Velocity Limit (RPM)
            theNode.Info.Ex.Parameter(98, 1);           //enable interrupting move
            cout << "AccLimit and VelLimit set" << endl;

            nodeList.push_back(&theNode);               // add node to list

            double timeout = myMgr->TimeStampMsec() + 2000; //TIME_TILL_TIMEOUT; //define a timeout in case the node is unable to enable
            //This will loop checking on the Real time values of the node's Ready status
            while (!theNode.Motion.IsReady()) {
                if (myMgr->TimeStampMsec() > timeout) {
                    printf("Error: Timed out waiting for Node %d to enable\n", iNode);
                    return -2;
                }
            }
        }
    }
    return 0;
}

void SolveCubicCoef(int loop_i){
    double sQ[6], Q[6];
    double dura = points[loop_i][6];
    // is it necessary to know the current position?
    
    for(int i = 0; i < 6; i++){
        sQ[i] = in1[i];
        Q[i] = points[loop_i][i];
            
        // solve coefficients of equations for cubic
        a[i] = sQ[i];
        b[i] = 3 / (dura * dura) * (Q[i] - sQ[i]);
        c[i] = -2 / (dura * dura * dura) * (Q[i] - sQ[i]);
    }
}

int SolveParaBlend(int loop_i, bool showAttention){
    // make them accessable from outside??
    float vMax[6] = {.4, .4, .4, 0.8, 0.8, 0.8}; // m/s, define the maximum velocity for each DoF
    float aMax[6] = {50, 50, 50, 10, 10, 10}; // m/s^2, define the maximum acceleration for each DoF
    double sQ[6], Q[6], o[6];
    double dura = points[loop_i][6];
    double unitV = sqrt(pow(points[loop_i][0] - in1[0], 2) + pow(points[loop_i][1] - in1[1], 2) + pow(points[loop_i][2] - in1[2], 2)); // the root to divide by to get unit vector
    
    for(int i = 0; i < 6; i++){
        sQ[i] = in1[i];
        Q[i] = points[loop_i][i];
        vMax[i] /= 1000; // change the velocity unit to meter per ms
        aMax[i] /= 1000000; // change the unit to meter per ms square
        tb[i] = i<3 ? dura - unitV / vMax[i] : dura - abs(Q[i] - sQ[i]) / vMax[i];
        if(tb[i] < 0) {
            cout << "WARNING: Intended trajectory exceeds velocity limit in DoF "<< i << ".\n";
            return -1;
        }
        else if (tb[i] > dura / 2){
            if (showAttention){ cout << "ATTENTION: Trajectory for DoF " << i << " will be in cubic form.\n"; }
            tb[i] = dura / 2;
            vMax[i] = 2 * (Q[i] - sQ[i]) / dura;
        }
        else if (i < 3) { vMax[i] = vMax[i] * (Q[i] - sQ[i]) / unitV; } // vMax in x,y,z accordingly
        else if (Q[i] < sQ[i]) { vMax[i] *= -1; } //Fix velocity direction for rotation
        o[i] = vMax[i] / 2 / tb[i];
        if(abs(o[i]*2) > aMax[i]){
            cout << "WARNING: Intended trajectory acceleration <" << abs(o[i]*2) << "> exceeds limit in DoF "<< i << ".\n";
            return -1;
        }
        
        // solve coefficients of equations for parabolic
        a[i] = sQ[i];
        b[i] = o[i];

        c[i] = sQ[i] - vMax[i] * tb[i] / 2;
        d[i] = vMax[i];

        e[i] = Q[i] - o[i] * dura * dura;
        f[i] = 2 * o[i] * dura;
        g[i] = -o[i];
    }
    return 0;
}

void RunTrajPoints(){
    char cmd;
    // Go through the given points
    for (int i = 0; i < points.size(); i++) {
        double t = 0;
        // trajectory generation and points splitting
        // SolveCubicCoef(i);
        if(SolveParaBlend(i, true) < 0) {
            cout << "Trajectory aborted.\n";
            return; 
        }
        cout << a[0] << ", " << b[0] << ", " << c[0] << ", " << d[0] << ", " << e[0] << ", " << f[0] << ", "<< g[0] << ", "<< tb[0] << endl;
        
        while (t <= points[i][6]){
            auto start = chrono::steady_clock::now();
            long dur = 0;
            
            // per time step pose
            // CUBIC equation
            // for (int j = 0; j < 6; j++){
            //     in1[j] = a[j] + b[j] * t * t + c[j] * t * t * t;
            // }
            // PARABOLIC BLEND equation
            for (int j = 0; j < 6; j++){
                if (t <= tb[j]){
                    in1[j] = a[j] + b[j] * t * t;
                }
                else if(t <= points[i][6]-tb[j]){
                    in1[j] = c[j] + d[j] * t;
                }
                else{
                    in1[j] = e[j] + f[j] * t + g[j] * t * t;
                }
            }
            // get absolute cable lengths in meters
            cout << "IN: "<< in1[0] << " " << in1[1] << " " << in1[2] << " " << in1[3] << " " << in1[4] << " " << in1[5] << endl;
            pose_to_length(in1, out1);
            cout << "OUT: "<<  out1[0] << "\t" << out1[1] << "\t" << out1[2] << "\t" << out1[3] << "\t" <<  out1[4] << "\t" << out1[5] << "\t" << out1[6] << "\t" << out1[7] << endl;
            
            // auto end = chrono::steady_clock::now();
            // dur = chrono::duration_cast<chrono::milliseconds>(end-start).count();
            // cout << "Step to command time: "<< dur << "\t";

            SendMotorGrp();

            auto end = chrono::steady_clock::now();
            dur = chrono::duration_cast<chrono::milliseconds>(end-start).count();
            // cout << "Before sleep: " << dur << endl;
            
            double dif = MILLIS_TO_NEXT_FRAME - dur - 1;
            if(dif > 0) { Sleep(dif);}
            // Sleep(MILLIS_TO_NEXT_FRAME);

            // end = chrono::steady_clock::now();
            // dur = chrono::duration_cast<chrono::milliseconds>(end-start).count();
            // cout << "Time elasped: " << dur << "\tIn-loop t: " << t << endl;
            t += MILLIS_TO_NEXT_FRAME;

            if(kbhit()){ // Emergency quit during trajectory control
                cout << "\nSystem interrupted!! Do you want to quit the trajectory control?\nq - Quit trajectory\nr - Resume trajectory\n";
                cin >> cmd;
                if(cmd == 'q' || cmd == 'Q'){
                    cout << "Trajectory control aborted.\n";
                    t = points[i][6];
                    i = points.size();
                    break;
                }
            }
        }
        cout << "----------Completed point " << i + 1 <<"----------" << endl;
    }
}

int32_t ToMotorCmd(int motorID, double length){
    double scale = 820632.006; //814873.3086; // 6400 encoder count per revoltion, 40 times gearbox, 50mm spool radias. ie 6400*40/(2*pi*0.05) 
    if (motorID == -1) { return length * scale; }
    return (length - offset[motorID]) * scale;
}

void SendMotorCmd(int n){
    // convert to absolute cable length command
    try{
        int32_t step = ToMotorCmd(n, out1[n]);
        nodeList[n]->Motion.MoveWentDone();
        nodeList[n]->Motion.MovePosnStart(step, true, true); // absolute position
        nodeList[n]->Motion.Adv.TriggerGroup(1);
    }
    catch(sFnd::mnErr& theErr) {    //This catch statement will intercept any error from the Class library
        cout << "\nERROR: Motor [" << n << "] command failed.\n";  
        printf("Caught error: addr=%d, err=0x%08x\nmsg=%s\n", theErr.TheAddr, theErr.ErrorCode, theErr.ErrorMsg);    
        logfile << "\nERROR: Motor [" << n << "] command failed.\n";  
        logfile << "Caught error: addr=" << theErr.TheAddr << ", err=" << theErr.ErrorCode << "\nmsg=" << theErr.ErrorMsg << endl;    
        logfile.close(); // Close the log file
    }
    // double trqMeas = nodeList[n]->Motion.TrqMeasured;
    // cout << "Node " << n << " " << step << "\tTorque: "<< trqMeas << endl;
    // cout << "\tPosition error: " << ((double)nodeList[n]->Motion.PosnMeasured-step) << endl;

    // should we put some saftely factor on tracking position error? set a flag
}

void SendMotorTrq(int n){
    //float tolerance = 0.1;
    // float Kp = 1000;

    // nodeList[n]->Motion.MoveWentDone();
    nodeList[n]->Motion.TrqCommanded.Refresh();
    float currentTorque = nodeList[n]->Motion.TrqCommanded.Value();
    // int moveSize = Kp * (targetTorque - currentTorque);
    // nodeList[n]->Motion.MovePosnStart(moveSize, false, true);
    // nodeList[n]->Motion.Adv.TriggerGroup(1);

    if(currentTorque > targetTorque){ nodeList[n]->Motion.MoveVelStart(-300); }
    else if (currentTorque < targetTorque - 1.8){ nodeList[n]->Motion.MoveVelStart(150); cout << "Too much torque!!\n";}
    else{ nodeList[n]->Motion.MoveVelStart(-10);}
    // printf("Node[%d], current torque: %f\tCommmanded step: %d\n", n, currentTorque, moveSize);
    printf("Node[%d], current torque: %f\n", n, currentTorque);

    // any safety measures for extremely large torques?
}

void SendMotorGrp(bool IsTorque){
    SysManager* myMgr = SysManager::Instance();
    IPort &myPort = myMgr->Ports(0);
    void (*func)(int){ SendMotorCmd };
    if(IsTorque){ func = SendMotorTrq; }
    else { logfile << in1[0] << ", " << in1[1] << ", " << in1[2] << ", " << in1[3] << ", " << in1[4] << ", " << in1[5] << endl; }

    thread nodeThreads[nodeNum];
    for(int i = 0; i < nodeNum; i++){
        nodeThreads[i] = thread((*func), i);
    }
    for(int i = 0; i < nodeNum; i++){
        nodeThreads[i].join();
    }
    myPort.Adv.TriggerMovesInGroup(1);
}

void TrjHome(){// !!! Define the task space velocity limit for homing !!!
    double velLmt = 0.05; // unit in meters per sec
    double dura = sqrt(pow(in1[0]-home[0],2)+pow(in1[1]-home[1],2)+pow(in1[2]-home[2],2))/velLmt*1000; // *1000 to change unit to ms
    double t = 0;
    cout << "Expected homing duration: " << dura <<"ms\n";
    if (dura == 0){ return; }

    for(int i = 0; i < 6; i++){
        // solve coefficients of equations for cubic
        a[i] = in1[i];
        b[i] = 3 / (dura * dura) * (home[i] - in1[i]);
        c[i] = -2 / (dura * dura * dura) * (home[i] - in1[i]);
    }
    while (t <= dura){
        auto start = chrono::steady_clock::now();
        long dur = 0;
        
        // CUBIC equation
        for (int j = 0; j < 6; j++){
            in1[j] = a[j] + b[j] * t * t + c[j] * t * t * t;
        }
        cout << "IN: "<< in1[0] << " " << in1[1] << " " << in1[2] << " " << in1[3] << " " << in1[4] << " " << in1[5] << endl;
        pose_to_length(in1, out1);
        // cout << "OUT: "<<  out1[0] << " " << out1[1] << " " << out1[2] << " " << out1[3] << endl;
        
        SendMotorGrp();
        
        auto end = chrono::steady_clock::now();
        dur = chrono::duration_cast<chrono::milliseconds>(end-start).count();
        cout << " Before sleep: " << dur << endl;
        
        double dif = MILLIS_TO_NEXT_FRAME - dur;
        if(dif > 0) { Sleep(dif); }
        // Sleep(MILLIS_TO_NEXT_FRAME);

        end = chrono::steady_clock::now();
        dur = chrono::duration_cast<chrono::milliseconds>(end-start).count();
        cout << " Time elasped: " << dur << "\tTime left: " << dura - t << endl;
        t += MILLIS_TO_NEXT_FRAME;
        if(kbhit()){ // Emergency quit during trajectory control
            cout << "\nSystem interrupted!! Do you want to quit the trajectory control?\nq - Quit trajectory\nr - Resume trajectory\n";
            char cmd;
            cin >> cmd;
            if(cmd == 'q' || cmd == 'Q'){
                cout << "Trajectory control aborted.\n";
                t = dura;
                break;
            }
        }
    }
    cout << "Homing with trajectory completed\n";
}

bool CheckLimits(){
    if(spcLimit.empty()){
        ifstream file ("limit.csv"); // Read limit file
        string temp;
        if(file.is_open()){
            while (file >> temp){
                spcLimit.push_back(stod(temp)); // convert string to double stod()
            }
            cout << "Completed reading external limit file" << endl;
        }
    }
    for (int i = 0; i < 3; i++){
        if(spcLimit[i*2]>in1[i] || in1[i]>spcLimit[i*2+1]){ return false; }
    }
    return true;
}