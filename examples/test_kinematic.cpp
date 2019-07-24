#include "Kinematics.h"
#include "Cia402device.h"
#include "CiA301CommPort.h"
#include "SocketCanPort.h"
#include <iostream>
#include "ToolsFControl.h"
#include "SerialArduino.h"

int main(){

    double dts=0.01;
    double incli, orient;

    ofstream graph("graph.csv",std::ofstream::out);

    //--imu sensor--
    SerialArduino tilt;
    float incSensor,oriSensor;


    //--Can port communications--
    SocketCanPort pm31("can1");
    SocketCanPort pm32("can1");
    SocketCanPort pm33("can1");

    CiA402SetupData sd31(2048,24,0.001, 0.144);
    CiA402SetupData sd32(2048,24,0.001, 0.144);
    CiA402SetupData sd33(2048,24,0.001, 0.144);

    CiA402Device m31 (31, &pm31, &sd31);
    CiA402Device m32 (32, &pm32, &sd32);
    CiA402Device m33 (33, &pm33, &sd33);

    //--Neck Kinematics--
    double l0=0.1090;
    double lg0=l0+0.002;
    GeoInkinematics neck_ik(0.052,0.052,l0); //kinematics geometric
    vector<double> lengths(3);
    double targetAngle1, targetAngle2, targetAngle3;
    double radio=0.0075;

    //    Motor setup
    m31.Reset();
    m31.SwitchOn();

    m32.Reset();
    m32.SwitchOn();

    m33.Reset();
    m33.SwitchOn();


    //set velocity and aceleration (rads/s)
    m31.SetupPositionMode(1,1);
    m32.SetupPositionMode(1,1);
    m33.SetupPositionMode(1,1);

    //
    sleep(4); //wait for sensor

    SamplingTime tools;
    tools.SetSamplingTime(dts);

    incli = 10;
    orient = 90;

    neck_ik.GetIK(incli,orient,lengths);
    targetAngle1=(lg0-lengths[0])/radio;//*180/(0.01*M_PI);
    targetAngle2=(lg0-lengths[1])/radio;//*180/(0.01*M_PI);
    targetAngle3=(lg0-lengths[2])/radio;//*180/(0.01*M_PI);

    m31.SetPosition(targetAngle1);
    m32.SetPosition(targetAngle2);
    m33.SetPosition(targetAngle3);


    for (double t=0;t<15;t+=dts)
    {
        tilt.readSensor(incSensor,oriSensor);
        cout <<"target1: "<<targetAngle1 <<"target2: "<<targetAngle2 <<"target3: "<<targetAngle3<<endl;
        cout <<"pos1: "<<m31.GetPosition()<<"pos2: "<<m32.GetPosition() <<"pos3: "<<m33.GetPosition()<<endl;
        cout << "incli_sen: " << incSensor << " , orient_sen: " << oriSensor << endl;
        graph << t << " , " << targetAngle1 << " , " << m31.GetPosition() << " , " << targetAngle2 << " , " << m32.GetPosition() << " , " << targetAngle3 << " , " << m33.GetPosition() << " , " << incli << " , " << incSensor << " , " << orient << " , " << oriSensor <<endl;

        tools.WaitSamplingTime();

    }

    sleep(2);

    m31.SetPosition(0.01);
    m32.SetPosition(0.01);
    m33.SetPosition(0.01);
    cout<<"FIN"<<endl;

    sleep(2);
    cout<<"FIN"<<endl;

}
