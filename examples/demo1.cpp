#include "Kinematics.h"
#include "Cia402device.h"
#include "CiA301CommPort.h"
#include "SocketCanPort.h"
#include <iostream>
#include "ToolsFControl.h"
#include "SerialArduino.h"


int main(){

    double dts=0.1;
    double incli, orient;

    //--sensor--
    SerialArduino tilt;
    float incSensor,oriSensor;
    ofstream graph("graph.csv",std::ofstream::out);


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
    double l0=0.1085;
    double lg0=l0+0.003;
    double radio=0.0075;
    GeoInkinematics neck_ik(0.052,0.052,l0); //kinematics geometric
    vector<double> lengths(3);
    double targetAngle1, targetAngle2, targetAngle3;

    sleep(4); //wait to imu sensor

    //    Motor setup
    m31.Reset();
    m31.SwitchOn();

    m32.Reset();
    m32.SwitchOn();

    m33.Reset();
    m33.SwitchOn();


    //set velocity and aceleration (rads/s)
    m31.SetupPositionMode(10,10);
    m32.SetupPositionMode(10,10);
    m33.SetupPositionMode(10,10);

    //

    SamplingTime tools;
    tools.SetSamplingTime(dts);

    incli = 0;
    orient = 1;
    //for (double k=0; k<2; k++){
    for(double i=0; i<1; i++){
        incli = incli+10;

    for(double j=1; j<35; j++){
        orient = orient+10;

    neck_ik.GetIK(incli,orient,lengths);
    targetAngle1=(lg0-lengths[0])/radio;//*180/(0.01*M_PI);
    targetAngle2=(lg0-lengths[1])/radio;//*180/(0.01*M_PI);
    targetAngle3=(lg0-lengths[2])/radio;//*180/(0.01*M_PI);

    m31.SetPosition(targetAngle1);
    m32.SetPosition(targetAngle2);
    m33.SetPosition(targetAngle3);


    for (double t=0;t<0.5;t+=dts)
    {
        cout <<"t: "<<t << endl;
        cout <<"target1: "<<targetAngle1;
        cout <<", target2: "<<targetAngle2;
        cout <<", target3: "<<targetAngle3<<endl;
        cout <<"pos1:    "<<m31.GetPosition();
        cout <<", pos2: "<<m32.GetPosition();
        cout <<", pos3: "<<m33.GetPosition()<<endl;
        cout << "orient: "<<orient<<" incl: "<<incli<<endl;
        tilt.readSensor(incSensor,oriSensor);
//        cout << "incli_sen: " << incSensor << " , orient_sen: " << oriSensor << endl;
//        graph << t << " , " << targetAngle1 << " , " << m31.GetPosition() << " , " << targetAngle2 << " , " << m32.GetPosition() << " , " << targetAngle3 << " , " << m33.GetPosition() << " , " << incli << " , " << incSensor << " , " << orient << " , " << oriSensor <<endl;

        tools.WaitSamplingTime();


    }

    }
    orient = 0;

    }

    //incli = 0;
    //}

    sleep(1);
        m31.SetPosition(0);
        m32.SetPosition(0);
        m33.SetPosition(0);
        sleep (1);

}

