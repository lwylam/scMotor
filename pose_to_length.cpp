#include "pose_to_length.h"
#include "Dependencies\eigen-3.3.7\Eigen\Dense"
#include <iostream>
#include <cmath>

using namespace Eigen;
using namespace std;

void pose_to_length(double pose[], double lengths[]){
    ///// Define the cable robot parameters here !!
    const int CABLE_NUM = 8;

    Vector3d frmOut[CABLE_NUM]; // coordinates of the fixed outlets on frame
    frmOut[0] << 0.0459, -3.7717, 1.6697;
    frmOut[1] << 0.0905, -3.7794, 0.0693;
    frmOut[2] << 0.0267, 0.0338, 1.6535;
    frmOut[3] << 0.0761, 0.0268, 0.0495;
    frmOut[4] << 2.7545, 0.0526, 1.6665;
    frmOut[5] << 2.7097, 0.0424, 0.0486;
    frmOut[6] << 2.7925, -3.7468, 1.6771;
    frmOut[7] << 2.7481, -3.7545, 0.0767;

    Vector3d frmOutUnitV[CABLE_NUM]; // unit vectors/directions of the fixed outlets on frame
    for(int i = 0; i < CABLE_NUM; i++){ frmOutUnitV[i] << 0, 0, 1; }

    Vector3d endOut[CABLE_NUM]; // local coordinates of cable outlets on end-effector, ie ^er_B
    endOut[0] << -0.12, -0.145, 0.05;
    endOut[1] << -0.12, -0.145, -0.05;
    endOut[2] << -0.12, 0.145, 0.05;
    endOut[3] << -0.12, 0.145, -0.05;
    endOut[4] << 0.12, 0.145, 0.05;
    endOut[5] << 0.12, 0.145, -0.05;
    endOut[6] << 0.12, -0.145, 0.05;
    endOut[7] << 0.12, -0.145, -0.05;

    const double pRadius = 0.025; // radius of rotating pulley on frame

    ///// End of manual model defination !!
    
    // local variables
    Vector3d orB[CABLE_NUM]; // vector from frame 0 origin to end effector cable outlet
    Vector3d orA[CABLE_NUM]; // vector from frame 0 origin to cable outlet point on rotating pulley
    Matrix3d oe_R; // rotation matrix from end effector to frame 0
    Matrix3d Ra, Rb, Rc;
    double a = pose[3], b = pose[4], c = pose[5];
    
    Ra << 1, 0, 0,
          0, cos(a), -sin(a),
          0, sin(a), cos(a);
    Rb << cos(b), 0,  sin(b),
          0, 1, 0,
          -sin(b), 0, cos(b);
    Rc << cos(c), -sin(c), 0, 
          sin(c), cos(c), 0, 
          0, 0, 1;
    oe_R = Ra * Rb * Rc; // Rotation matrix from given pose rotation
    Vector3d endEfr(pose[0], pose[1], pose[2]); // Translation of end-effector
    for(int i = 0; i < CABLE_NUM; i++){
        ///// Calculate orB[8] vectors /////
        orB[i] = oe_R*endOut[i] + endEfr;
                
        ///// Calculate cable outlet point on rotating pulley, ie point A /////
        Vector3d plNormal = frmOutUnitV[i].cross(orB[i] - frmOut[i]); // normal vector of the plane that the rotating pulley is in
        Vector3d VecC = plNormal.cross(frmOutUnitV[i]); // direction from fixed point towards pulley center
        Vector3d orC = VecC/VecC.norm()*pRadius + frmOut[i]; // vector from frame 0 origin to rotating pulley center
        double triR = pRadius / (orB[i] - orC).norm(); // triangle ratio??
        orA[i] = orC + triR*triR*(orB[i] - orC) - triR*sqrt(1 - triR*triR)*((plNormal/plNormal.norm()).cross(orB[i] - orC));
        
        ///// Calculate arc length /////
        Vector3d UVecCF = -VecC / VecC.norm(); // unit vector of CF
        Vector3d UVecCA = (orA[i]- orC) / (orA[i]- orC).norm(); // unit vector of CA
        double l_arc = pRadius * acos(UVecCF.dot(UVecCA));

        ///// Sum the total cable length /////
        lengths[i] = l_arc + (orA[i] - orB[i]).norm();
    }
}