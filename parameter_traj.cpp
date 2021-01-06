#include "parameter_traj.h"
// #include "Dependencies\eigen-3.3.7\Eigen\Dense"
#include <cmath>
#include <iostream>
#include <vector>

// using namespace Eigen;
using namespace std;

double tempPt[7];
double currentP1[3];
double currentP2[3];
double plateLast[3];
// bool reverse = false;
bool lastMove = false;

void parameter_traj(vector<vector<double>>& points, double startPos[]){
/////////////////// Define the tank parameters here ///////////////////
    double tankStart[3] = {2.198, -2.8314, 0.463}; // poke with end effector
    double tankX[3] = {2.205, -2.8314, 0.463}; // poke with end effector (end of cleaning area)
    double tankY[3] = {2.185, -3.6644, 0.463}; // poke with end effector (end of cleaning area)
    double tankDepth = -0.31; // meter, define depth from experience?

    double plateInclination = 0.959931; // radian, 55 degree, from horizon
    // Vector3d plateDistance(2.237, -3.486, 0); // addition going to next plate start
    int plateNum = 8; // number of plates to clean
    
    double endEffectorWidth = 0.2; // meter
    // float overlapPercentage = 0.1;
    
    double safeHeight = 0.1; // meter, safety height above tank
    double leaveTankHeight = 0.5; // meter, saftey height above tank for start and end of traj
    double velLmt = 0.05; // unit in meters per sec
    double zeroRotation[3] = {0,0,0};
    int brushTimes = 1;
/////////////////// End of tank parameters defination ///////////////////
    
    points.clear(); // Clear the list to start from new
    
    if(startPos[2] < tankStart[2] + safeHeight){ cout << "\nWarning! End-effector still inside tank!\n"; return; }

    double uX = sqrt(pow(tankX[0]-tankStart[0],2)+pow(tankX[1]-tankStart[1],2)+pow(tankX[2]-tankStart[2],2));
    double unitX[3] = {(tankX[0]-tankStart[0])/uX, (tankX[1]-tankStart[1])/uX, (tankX[2]-tankStart[2])/uX};
    // double uY = sqrt(pow(tankY[0]-tankStart[0],2)+pow(tankY[1]-tankStart[1],2)+pow(tankY[2]-tankStart[2],2));
    // double unitY[3] = {(tankY[0]-tankStart[0])/uY, (tankY[1]-tankStart[1])/uY, (tankY[2]-tankStart[2])/uY};
    double unitPlate[3] = {(tankY[0]-tankStart[0])/(plateNum-1), (tankY[1]-tankStart[1])/(plateNum-1), (tankY[2]-tankStart[2])/(plateNum-1)};
    for(double val : unitX){ cout << val << "\t"; }
    cout << "unit vector of X\n";
    for(double val : unitPlate){ cout << val << "\t"; } 
    cout << "unit vector of Plate\n";
    int moveNumP = (tankX[0]-tankStart[0])/endEffectorWidth/unitX[0] + 1; // number of cleaning cycle on one plate, including start and end pose
    if(moveNumP < 1){ moveNumP = 1; } // Safety, brush at least once
    double leftover = remainder(tankX[0]-tankStart[0], endEffectorWidth*unitX[0]);
    lastMove = leftover < 0 || leftover > 0.01; // check if reminder is present
    cout << "moveNumP = " << moveNumP << ", last move = " << lastMove << ", leftover = " << leftover << endl;

    safeHeight /= 2;
    double plateT = abs(tankDepth)/velLmt*1000;
    double safeT = safeHeight/velLmt*1000*2; // slower safe height movements
    copy(begin(zeroRotation), end(zeroRotation), tempPt+3); // set the rotation as zero for all time?
    
    // from current position to start pose
    tempPt[0] = tankStart[0];
    tempPt[1] = tankStart[1] + safeHeight*tan(plateInclination);
    tempPt[2] = tankStart[2] + safeHeight + leaveTankHeight;
    tempPt[6] = sqrt(pow(startPos[0]-tempPt[0],2)+pow(startPos[1]-tempPt[1],2)+pow(startPos[2]-tempPt[2],2))/velLmt*1000; // *1000 to change unit to ms
    points.push_back(vector<double>(tempPt, tempPt+7)); // safety leave tank height above start point
    tempPt[2] -= leaveTankHeight; // real safety height above tank
    tempPt[6] = leaveTankHeight/velLmt*1000;
    points.push_back(vector<double>(tempPt, tempPt+7)); // safety height above start point
    copy(begin(tankStart), end(tankStart), begin(tempPt));
    tempPt[6] = safeT;
    points.push_back(vector<double>(tempPt, tempPt+7)); // start point
    
    copy(begin(tankStart), end(tankStart), begin(currentP1)); // start of plate first move
    copy(begin(tankStart), end(tankStart), begin(currentP2)); // start of first plate
    copy(begin(tankX), end(tankX), begin(plateLast)); // plate last move
    for(int p = 0; p < plateNum; p++){ // diff plates
        for (int i = 0; i < moveNumP; i++){ // within same plate
            // go down and up
            for (int b = 0; b < brushTimes; b++){
                tempPt[2] = currentP1[2] + tankDepth; // z-axis
                tempPt[1] = currentP1[1] - tankDepth/tan(plateInclination); // y-axis
                tempPt[6] = plateT;
                points.push_back(vector<double>(tempPt, tempPt+7)); // plate bottom
                copy(begin(currentP1), end(currentP1), begin(tempPt));
                points.push_back(vector<double>(tempPt, tempPt+7)); // plate top
            }

            // next position start
            currentP1[0] += unitX[0]*endEffectorWidth;
            currentP1[1] += unitX[1]*endEffectorWidth;
            // // safty height
            // tempPt[0] = currentP1[0];
            // tempPt[1] = currentP1[1] + safeHeight*tan(plateInclination);
            // tempPt[2] = currentP1[2] + safeHeight;
            // tempPt[6] = safeT*2;
            // points.push_back(vector<double>(tempPt, tempPt+7)); // next plate safty height
            if(i == moveNumP - 1){
                if(!lastMove){
                    tempPt[2] += safeHeight;
                    tempPt[6] = safeT;
                    points.push_back(vector<double>(tempPt, tempPt+7)); // staight plate safty height
                }
                continue;
            }
            copy(begin(currentP1), end(currentP1), begin(tempPt));
            tempPt[6] = endEffectorWidth/velLmt*1000*2; // move by one endEffectorWidth time
            points.push_back(vector<double>(tempPt, tempPt+7)); // plate top
        }
        if(lastMove){
            copy(begin(plateLast), end(plateLast), begin(tempPt));
            tempPt[6] = safeT;
            points.push_back(vector<double>(tempPt, tempPt+7)); // plate last top

            for (int b = 0; b < brushTimes; b++){
                tempPt[2] += tankDepth; // z-axis
                tempPt[1] -= tankDepth/tan(plateInclination); // y-axis
                tempPt[6] = plateT;
                points.push_back(vector<double>(tempPt, tempPt+7)); // plate bottom
                copy(begin(plateLast), end(plateLast), begin(tempPt));
                points.push_back(vector<double>(tempPt, tempPt+7)); // plate top
            }
            tempPt[1] += safeHeight*tan(plateInclination);
            tempPt[2] += safeHeight;
            tempPt[6] = safeT;
            points.push_back(vector<double>(tempPt, tempPt+7)); // plate safty height
            // update next last move
            plateLast[0] += unitPlate[0];
            plateLast[1] += unitPlate[1];
        }
        if(p == plateNum - 1){ continue; }
        // next plate start
        currentP2[0] += unitPlate[0];
        currentP2[1] += unitPlate[1];
        tempPt[6] = sqrt(pow(currentP2[0]-currentP1[0],2)+pow(currentP2[1]-currentP1[1],2))/velLmt*1000*1.8; // calculate time first
        copy(begin(currentP2), end(currentP2), begin(currentP1));
        // next safty height
        tempPt[0] = currentP1[0];
        tempPt[1] = currentP1[1] + safeHeight*tan(plateInclination);
        tempPt[2] = currentP1[2] + safeHeight;
        points.push_back(vector<double>(tempPt, tempPt+7)); // next plate safty height
        copy(begin(currentP1), end(currentP1), begin(tempPt));
        tempPt[6] = safeT;
        points.push_back(vector<double>(tempPt, tempPt+7)); // plate top
    }
    // Ending point to leave completely from tank
    tempPt[2] += leaveTankHeight;
    tempPt[6] = leaveTankHeight/velLmt*1000;
    points.push_back(vector<double>(tempPt, tempPt+7)); // plate top
}