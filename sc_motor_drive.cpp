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
void SolveCubicCoef(int loop_i);
int32_t ToMotorCmd(int motorID, double length);

vector<string> comHubPorts;
vector<INode*> nodeList; // create a list for each node
vector<vector<double>> points;
unsigned int portCount;
const int MILLIS_TO_NEXT_FRAME = 20;   
double in1[6] = {2, 0.5, 2, 0, 0, 0};
double out1[4];
double a0, a2, a3, b0, b2, b3, c0, c2, c3, d0, d2, d3, e0, e2, e3, f0, f2, f3;

int main()
{h

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
        cout << "Completed reading external input file\n";
    }
    
    // Go through the given points
    for (int i = 0; i < points.size(); i++){
        double t = 0;
        // trajectory generation and points splitting
        SolveCubicCoef(i);
       
        while (t <= points[i][6]){
            auto start = chrono::steady_clock::now();
            long dur = 0;
            // per time step pose
            in1[0] = a0 + a2 * t * t + a3 * t * t * t;
            in1[1] = b0 + b2 * t * t + b3 * t * t * t;
            in1[2] = c0 + c2 * t * t + c3 * t * t * t;
            in1[3] = d0 + d2 * t * t + d3 * t * t * t;
            in1[4] = e0 + e2 * t * t + e3 * t * t * t;
            in1[5] = f0 + f2 * t * t + f3 * t * t * t;
            // get absolute cable lengths in meters
            cout << in1[0] << " " << in1[1] << " " << in1[2] << " " << in1[3] << " " << in1[4] << " " << in1[5] << endl;
            compile_lengths(in1, out1);
            cout << out1[0] << " " << out1[1] << " " << out1[2] << " " << out1[3] << endl;

            bool allDone = false;
            vector<double> timeout;
            for (int n = 0; n<myPort.NodeCount(); n++) { 
                // convert to absolute cable length command
                int32_t step = ToMotorCmd(n, out1[n]);
                nodeList[n]->Motion.MoveWentDone();
                nodeList[n]->Motion.MovePosnStart(step, true, true); // absolute position
                timeout.push_back(nodeList[n]->Motion.MovePosnDurationMsec(step, true)); // absolute position
                nodeList[n]->Motion.Adv.TriggerGroup(1);
                cout << "Node " << n << " " << step << "\t Torque: "<< (double)nodeList[n]->Motion.TrqMeasured << endl;
            }
            cout << endl;
            // trigger motors to move
            myPort.Adv.TriggerMovesInGroup(1);
            double t_max =  myMgr->TimeStampMsec() + *max_element(timeout.begin(), timeout.end()) + 100;
            cout << "Estimated time: " << t_max;

            Sleep(MILLIS_TO_NEXT_FRAME); // need some time before sending the next position command? Or else error occurs?
            
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
            auto end = chrono::steady_clock::now();
            dur = chrono::duration_cast<chrono::milliseconds>(end-start).count();
            cout << " Time elasped: " << dur << " ; In-loop t: " << t << endl;
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

    for(int i = 0; i < nodeList.size(); i++){ //Disable Nodes
        myPort.Nodes(i).EnableReq(false);
    }
    nodeList.clear();
    myMgr->PortsClose(); // Close down the ports
    
    // q_initial="2 0.5 2 0 0 0" q_min="0 0 0 -3.1415 -3.1415 -3.1415" q_max="5.0 1.0 5.0 3.1416 3.1416 3.1416"
    // compile_lengths(in1, out1);

    /*
        //At this point we will execute 10 rev moves sequentially on each axis
        
        //After moves have completed Disable node, and close ports
        printf("Disabling nodes, and closing this port\n");
        for (size_t iNode = 0; iNode < myPort.NodeCount(); iNode++) {
            myPort.Nodes(iNode).EnableReq(false); //Disable Nodes
        }
        nodeList.clear();
    */

    cout << "\n" << "Done" << endl;
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
            theNode.Motion.VelLimit = 2500;             //700 Set Velocity Limit (RPM)
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
    float sA1, sA2, sA3, sA4, sA5, sA6, A1, A2, A3, A4, A5, A6;
    double home[] = {2, 0.5, 2, 0, 0, 0}; // home posisiton
    // is it necessary to know the current position?
    
    sA1 = loop_i == 0 ? home[0] : points[loop_i - 1][0];
    sA2 = loop_i == 0 ? home[1] : points[loop_i - 1][1];
    sA3 = loop_i == 0 ? home[2] : points[loop_i - 1][2];
    sA4 = loop_i == 0 ? home[3] : points[loop_i - 1][3];
    sA5 = loop_i == 0 ? home[4] : points[loop_i - 1][4];
    sA6 = loop_i == 0 ? home[5] : points[loop_i - 1][5];
    A1 = points[loop_i][0];
    A2 = points[loop_i][1];
    A3 = points[loop_i][2];
    A4 = points[loop_i][3];
    A5 = points[loop_i][4];
    A6 = points[loop_i][5];

    // solve coefficients of equations for cubic
    double d = points[loop_i][6];
    a0 = sA1;
    a2 = 3 / (d * d) * (A1 - sA1);
    a3 = -2 / (d * d * d) * (A1 - sA1);
    b0 = sA2;
    b2 = 3 / (d * d) * (A2 - sA2);
    b3 = -2 / (d * d * d) * (A2 - sA2);
    c0 = sA3;
    c2 = 3 / (d * d) * (A3 - sA3);
    c3 = -2 / (d * d * d) * (A3 - sA3);
    d0 = sA4;
    d2 = 3 / (d * d) * (A4 - sA4);
    d3 = -2 / (d * d * d) * (A4 - sA4);
    e0 = sA5;
    e2 = 3 / (d * d) * (A5 - sA5);
    e3 = -2 / (d * d * d) * (A5 - sA5);
    f0 = sA6;
    f2 = 3 / (d * d) * (A6 - sA6);
    f3 = -2 / (d * d * d) * (A6 - sA6);
}

int32_t ToMotorCmd(int motorID, double length){
    double offset = -5.5; // dummy value, the offset from "zero position"
    double scale = 814873.3086; // 6400 encoder count per revoltion, 40 times gearbox, 50mm spool radias. ie 6400*40/(2*pi*0.05)
    switch (motorID){ // only 4 motors are assumed to present
    case 0:
        offset = -6.43438;
        break;
    case 1:
        offset = -7.67390;
        break;
    case 2:
        offset = -6.06174;
        break;
    case 3:
        offset = -4.48486;
        break;
    
    default:
        break;
    }
    return (length + offset) * scale;
}