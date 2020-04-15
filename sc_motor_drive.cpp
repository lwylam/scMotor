#include <vector>
#include <string>
#include <fstream>
#include <iostream>
#include <sstream>
#include <algorithm>
#include <chrono>
#include <thread>
#include <stdio.h>
#include <windows.h>
#include "compile_lengths.h"
#include "Dependencies\sFoundation20\inc\pubSysCls.h"

using namespace std;
using namespace sFnd;

int CheckMotorNetwork();
void SendMotorCmd(int n);
void SolveCubicCoef(int loop_i);
int SolveParaBlend(int loop_i, bool showAttention = false);
int32_t ToMotorCmd(int motorID, double length);

vector<string> comHubPorts;
vector<INode*> nodeList; // create a list for each node
vector<vector<double>> points;
vector<double> timeout;
unsigned int portCount;
const int MILLIS_TO_NEXT_FRAME = 20; // note the basic calculation time is abt 16ms
const double home[] = {2, 0.5, 2, 0, 0, 0}; // home posisiton
double in1[6] = {2, 0.5, 2, 0, 0, 0};
double out1[4] = {6.43438, 7.6739, 6.06174, 4.48486};
double a[6], b[6], c[6], d[6], e[6], f[6], g[6], tb[6]; // trajectory coefficients

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

    // if motors's states are right, do some current mode and homing here
    {
        bool allDone = false;
        for (int n = 0; n<myPort.NodeCount(); n++) { 
            nodeList[n]->Motion.MoveWentDone();
            nodeList[n]->Motion.MovePosnStart(0, true); // absolute position
        }
        while(!allDone) {
            for (INode* n : nodeList) {
                if(!n->Motion.MoveIsDone()) {
                    break;
                }
                allDone = true;
            }
        }
        cout << "Homing completed" << endl;
        Sleep(2000);
    }
    
    //myNode.Motion.AddToPosition(-myNode.Motion.PosnMeasured); // SAMPLE: Zeroing the number space around the current Measured Position
    
    // Read input file for traj-gen
    ifstream file ("input.csv");  // TODO: invalid read?
    vector<double> row;
    string line, word, temp;
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
    
    // Go through the given points
    for (int i = 0; i < points.size(); i++){
        double t = 0;
        // trajectory generation and points splitting
        // SolveCubicCoef(i);
        if(SolveParaBlend(i) < 0){ return -3; }
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
                    in1[j] = c[j] + d[j];
                }
                else{
                    in1[j] = e[j] + f[j] * t + g[j] * t * t;
                }
            }
            // get absolute cable lengths in meters
            cout << "IN: "<< in1[0] << " " << in1[1] << " " << in1[2] << " " << in1[3] << " " << in1[4] << " " << in1[5] << endl;
            // q_initial="2 0.5 2 0 0 0" q_min="0 0 0 -3.1415 -3.1415 -3.1415" q_max="5.0 1.0 5.0 3.1416 3.1416 3.1416"
            compile_lengths(in1, out1);
            cout << "OUT: "<<  out1[0] << " " << out1[1] << " " << out1[2] << " " << out1[3] << endl;
            
            auto end = chrono::steady_clock::now();
            dur = chrono::duration_cast<chrono::milliseconds>(end-start).count();
            cout << "Step to command time: "<< dur << "\t";

            // thread th4(SendMotorCmd,3);
            // thread th3(SendMotorCmd,2);
            thread th2(SendMotorCmd,1);
            thread th1(SendMotorCmd,0);
            
            //bool allDone = false;
            
            // double t_max =  myMgr->TimeStampMsec() + *max_element(timeout.begin(), timeout.end()) + 100;
            // cout << "Estimated time: " << t_max;
            end = chrono::steady_clock::now();
            dur = chrono::duration_cast<chrono::milliseconds>(end-start).count();
            cout << " Before sleep: " << dur << endl;
            
            // double dif = MILLIS_TO_NEXT_FRAME - dur - 2;
            // if(dif > 0) { Sleep(dif);}
            Sleep(MILLIS_TO_NEXT_FRAME);

            // if(*max_element(timeout.begin(), timeout.end())>MILLIS_TO_NEXT_FRAME){
            //     cout << "Cannot complete motion in given duration. Exit programme." << endl;
            //     return -1;
            // }
            
            // wait for respond
            /*while(!allDone) { //&& dur < MILLIS_TO_NEXT_FRAME
                auto end = chrono::steady_clock::now();
                dur = chrono::duration_cast<chrono::milliseconds>(end-start).count();

                if(!allDone){
                    if (myMgr->TimeStampMsec() > t_max) {
                        cout << "Error: Timed out waiting for move to complete" << endl;
                        return -2;
                    }
                    for (INode* n : nodeList) {
                        if(!n->Motion.IsReady()) { // or .MoveIsDone?
                            //allDone = false;
                            break;
                        }
                        allDone = true;
                    } 
                }
            }*/

            // wait sending motor command to complete before triggering motors to move
            // th4.join();
            // th3.join();
            th2.join();
            th1.join();
            
            myPort.Adv.TriggerMovesInGroup(1);
            end = chrono::steady_clock::now();
            dur = chrono::duration_cast<chrono::milliseconds>(end-start).count();
            cout << " Time elasped: " << dur << "\tIn-loop t: " << t << endl;
            timeout.clear();
            t += MILLIS_TO_NEXT_FRAME;
        }
        cout << "----------Completed point " << i <<"----------" << endl;
        // loop over again until the entire motion is completed? exit programme
    }

    // Test velocity mode
    /*for (int n = 0; n<myPort.NodeCount(); n++) { 
        double vel = 600;
        cout << "Node " << n << " start velocity mode: "<< vel << endl;
        cout << "Expected time: " << nodeList[n]->Motion.MoveVelDurationMsec(vel) << endl;
        
        auto start = chrono::steady_clock::now();
        nodeList[n]->Motion.MoveVelStart(vel);
        while(!nodeList[n]->Motion.VelocityAtTarget()){}
        auto end = chrono::steady_clock::now();
        auto dur = chrono::duration_cast<chrono::milliseconds>(end-start).count();
        cout << "Time elapsed: " << dur << endl;
        for(int i = 0; i < 10; i++){
            double pos1 = nodeList[n]->Motion.PosnMeasured;
            Sleep(98); //100-2
            double pos2 = nodeList[n]->Motion.PosnMeasured;
            cout << "Measured velocity: " << (pos2-pos1)/0.1/6400*60 << " RPM" << endl;
            //Sleep(500);
        }
        auto start_p = chrono::steady_clock::now();
        cout << "Teknic report vel: " << (double)nodeList[n]->Motion.VelMeasured << endl;
        //double pos = nodeList[n]->Motion.PosnMeasured;
        auto end_p = chrono::steady_clock::now();
        auto dur_p = chrono::duration_cast<chrono::milliseconds>(end_p-start_p).count();
        nodeList[n]->Motion.MoveVelStart(0);
        cout << "Velocity set 0\tRequest velocity time: " << dur_p << endl;
    } */

    // Test torque control frequency
    for (int n = 0; n < nodeList.size(); n++){
        auto previousT = chrono::steady_clock::now();
        long dur = 0;
        double targetTrq = 3;
        double targetVel;
        double Kq = 5;
        
        cout << "Node " << n << " start torque test" << endl;
        for (int i = 0; i < 30; i++){
            nodeList[n]->Motion.TrqMeasured.Refresh();
			float motorCur = nodeList[n]->Motion.TrqMeasured.Value();
            targetVel = -Kq * (targetTrq + motorCur);
            printf("Current size: \t%f \t%f \n", motorCur, targetVel);
            nodeList[n]->Motion.Adv.MoveVelStart(targetVel);
            myMgr->Delay(nodeList[n]->Motion.Adv.MoveVelDurationMsec(targetVel));
            if (motorCur < -15) { //safety measure
                nodeList[n]->Motion.Adv.MoveVelStart(0);
                break;
            }
            // int multiplier = 100;
            // int actualMove = 10;
            // int moveSize = 10;
            // nodeList[n]->Motion.TrqCommanded.Refresh();
            // float currentTorque = nodeList[n]->Motion.TrqCommanded.Value();
            // actualMove = moveSize * abs(currentTorque);
            // cout << actualMove << endl;
            // if (currentTorque < 3) {
            //     nodeList[n]->Motion.MovePosnStart(actualMove+ moveSize * multiplier);
            //     //myMgr->Delay(nodeList[n]->Motion.MovePosnDurationMsec(actualMove + moveSize * multiplier));
            //     cout << "adding " << actualMove << endl;
            //     cout << "Moving time delay: " << nodeList[n]->Motion.MovePosnDurationMsec(actualMove * 100) << endl;
            // }
            // else {
            //     nodeList[n]->Motion.MovePosnStart(-1* actualMove - moveSize * multiplier);
            //     //myMgr->Delay(nodeList[n]->Motion.MovePosnDurationMsec(-1 * actualMove - moveSize * multiplier));
            //     cout << "subtracting " << actualMove << endl;
            //     cout << "Moving time delay: " << nodeList[n]->Motion.MovePosnDurationMsec(actualMove * 100) << endl;
            // }
            // printf("Current torque: \t%8.0f \n", currentTorque);

            // show frequency
            auto now = chrono::steady_clock::now();
            dur = chrono::duration_cast<chrono::milliseconds>(now-previousT).count();
            previousT = now;
            cout << "Loop time: "<< dur << endl;
        }
        nodeList[n]->Motion.Adv.MoveVelStart(0);
        cout << "Node " << n << " complete torque test\n" << endl;
        Sleep(2000);
    }
    
    Sleep(6000); // wait a little more before disabling the nodes
    for(int i = 0; i < nodeList.size(); i++){ //Disable Nodes
        myPort.Nodes(i).EnableReq(false);
    }
    nodeList.clear();
    myMgr->PortsClose(); // Close down the ports
    
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
        sQ[i] = loop_i == 0 ? home[i] : points[loop_i - 1][i];
        Q[i] = points[loop_i][i];
            
        // solve coefficients of equations for cubic
        a[i] = sQ[i];
        b[i] = 3 / (dura * dura) * (Q[i] - sQ[i]);
        c[i] = -2 / (dura * dura * dura) * (Q[i] - sQ[i]);
    }
}

int SolveParaBlend(int loop_i, bool showAttention){
    // make them accessable from outside??
    float vMax[6] = {.01, .001, .01, 0.8, 0.8, 0.8}; // Define the maximum velocity for each DoF
    float aMax[6] = {.01, .01, .01, .01, .01, .01}; // Define the maximum acceleration for each DoF
    double sQ[6], Q[6], o[6];
    double dura = points[loop_i][6];
    
    for(int i = 0; i < 6; i++){
        sQ[i] = loop_i == 0 ? home[i] : points[loop_i - 1][i];
        Q[i] = points[loop_i][i];
        tb[i] = dura - (Q[i] - sQ[i]) / vMax[i];
        if(tb[i] < 0) {
            cout << "WARNING: Intended trajectory exceeds velocity limit in DoF "<< i << ".\n";
            return -1;
        }
        else if (tb[i] > dura / 2){
            if (showAttention){ cout << "ATTENTION: Trajectory for DoF " << i << " will be in cubic form.\n"; }
            tb[i] = dura / 2;
            vMax[i] = (Q[i] - sQ[i]) / dura;
        }
        o[i] = vMax[i] / 2 / tb[i];
        if(abs(o[i]*2) > aMax[i]){
            cout << "WARNING: Intended trajectory exceeds acceleration limit in DoF "<< i << ".\n";
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

int32_t ToMotorCmd(int motorID, double length){
    double offset[4] = {6.43438, 7.67390, 6.06174, 4.48486}; // length to offset, from "zero position"
    double scale = 814873.3086; // 6400 encoder count per revoltion, 40 times gearbox, 50mm spool radias. ie 6400*40/(2*pi*0.05)
    
    return (length - offset[motorID]) * scale;
}

void SendMotorCmd(int n){
    // convert to absolute cable length command
    int32_t step = ToMotorCmd(n, out1[n]);
    nodeList[n]->Motion.MoveWentDone();
    nodeList[n]->Motion.MovePosnStart(step, true, true); // absolute position
    timeout.push_back(nodeList[n]->Motion.MovePosnDurationMsec(step, true)); // absolute position
    nodeList[n]->Motion.Adv.TriggerGroup(1);
    cout << "Node " << n << " " << step << "\tTorque: "<< (double)nodeList[n]->Motion.TrqMeasured;
    cout << "\tPosition error: " << ((double)nodeList[n]->Motion.PosnMeasured-step) << endl;

    // should we put some saftely factor on tracking position error? set a flag
}