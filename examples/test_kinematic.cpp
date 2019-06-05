#include "Kinematics.h"
#include "Cia402device.h"
#include "CiA301CommPort.h"
#include "SocketCanPort.h"
#include <iostream>
#include "ToolsFControl.h"

int main(){

    double dts=0.01;
    double incli, orient;

    //--Can port communications--
    SocketCanPort pm1("can1");
    SocketCanPort pm2("can1");
    SocketCanPort pm3("can1");

    CiA402SetupData sd1(4096,3.7,0.001,4);
    CiA402SetupData sd2 = sd1;
    CiA402SetupData sd3 = sd1;

    CiA402Device m1 (1, &pm1, &sd1);
    CiA402Device m2 (2, &pm2, &sd2);
    CiA402Device m3 (3, &pm3, &sd3);

    //--Neck Kinematics--
    double l0=0.1090;
    double lg0=l0+0.002;
    GeoInkinematics neck_ik(0.052,0.052,l0); //kinematics geometric
    vector<double> lengths(3);
    double targetAngle1, targetAngle2, targetAngle3;

    //    Motor setup
    m1.Reset();
    m1.SwitchOn();

    m2.Reset();
    m2.SwitchOn();

    m3.Reset();
    m3.SwitchOn();

          //set velocity and aceleration (rads/s)
    m1.SetupPositionMode(1,1);
    m2.SetupPositionMode(1,1);
    m3.SetupPositionMode(1,1);
    //

    ToolsFControl tools;
    tools.SetSamplingTime(dts);

    incli = 14.5;
    orient = 300.8;

    neck_ik.GetIK(incli,orient,lengths);
    targetAngle1=(lg0-lengths[0])/0.01;//*180/(0.01*M_PI);
    targetAngle2=(lg0-lengths[1])/0.01;//*180/(0.01*M_PI);
    targetAngle3=(lg0-lengths[2])/0.01;//*180/(0.01*M_PI);

    m1.SetPosition(targetAngle1);
    m2.SetPosition(targetAngle2);
    m3.SetPosition(targetAngle3);

    cout <<"target1: "<<targetAngle1<<" target2: "<<targetAngle2<<" target3: "<<targetAngle3<<endl;

    for (double t=0;t<5;t+=dts)
    {
        cout <<"pos1: "<<m1.GetPosition()<<" pos2: "<<m2.GetPosition()<<" pos3: "<<m3.GetPosition()<<endl;
        tools.WaitSamplingTime();

    }

}
