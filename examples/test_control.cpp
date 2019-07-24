#include "Kinematics.h"
#include "Cia402device.h"
#include "CiA301CommPort.h"
#include "SocketCanPort.h"
#include <iostream>
#include "ToolsFControl.h"
#include "fcontrol.h"
#include "SerialArduino.h"



int main(){

    double dts=0.01;

    //--sensors--
    SerialArduino tilt;
    float incSensor,oriSensor;


    //--Controllers--
    vector<double> npi ={0.2916 ,  -5.3981 ,   0.2415  ,  5.5243};
    vector<double> dpi ={0.5882 , -0.9921 , -0.5804 , 1.0000};
    SystemBlock internal1(npi,dpi);
    SystemBlock internal3(npi,dpi);
    SystemBlock internal2(npi,dpi);

    string method("w10p60pid"); // entero
    PIDBlock external1(1,0,0.1,dts);
    PIDBlock external2(1,0,0.1,dts);
    PIDBlock external3(1,0,0.1,dts);

    //--Can port communications--
    SocketCanPort pm1("can1");
    SocketCanPort pm2("can1");
    SocketCanPort pm3("can1");

    CiA402SetupData sd1(4096,3.7,0.001,4);
    CiA402SetupData sd2 = sd1;
    CiA402SetupData sd3 = sd1;

    CiA402Device m1 (31, &pm1, &sd1);
    CiA402Device m2 (32, &pm2, &sd2);
    CiA402Device m3 (33, &pm3, &sd3);

    //--Neck Kinematics--
    double l0=0.1090;
    double lg0=l0+0.002;
    GeoInkinematics neck_ik(0.052,0.052,l0); //kinematics geometric
    vector<double> lengths(3);

    //vars
    double incli, orient;
    double ep1,ev1,cs1;
    double ep2,ev2,cs2;
    double ep3,ev3,cs3;
    double posan1, posan2, posan3;

    //    Motor setup
    m1.Reset();
    m1.SwitchOn();

    m2.Reset();
    m2.SwitchOn();

    m3.Reset();
    m3.SwitchOn();

    m1.Setup_Torque_Mode();
    m2.Setup_Torque_Mode();
    m3.Setup_Torque_Mode();

    //

    SamplingTime tools;
    tools.SetSamplingTime(dts);
    sleep(4);

    incli = 20;
    orient = 90;

    for (double t=0;t<3;t+=dts){

        tilt.readSensor(incSensor,oriSensor);
        cout  << "incli " << incli << ",  orient " << orient  << endl;
        cout << "incli_sen: " << incSensor << " , orient_sen: " << oriSensor <<  endl;

        neck_ik.GetIK(incli,orient,lengths);
        posan1=(lg0-lengths[0])/0.01;//*180/(0.01*M_PI);
        posan2=(lg0-lengths[1])/0.01;//*180/(0.01*M_PI);
        posan3=(lg0-lengths[2])/0.01;//*180/(0.01*M_PI);

        cout << "TARGET: , " << posan1  << " , " << posan2 << " , " << posan3 << endl;


        ep1=posan1-m1.GetPosition();
        cs1=ep1 > external1;
        ev1= cs1-m1.GetVelocity();
        m1.SetTorque((ev1 > internal1));

        ep2=posan2-m2.GetPosition();
        cs2=ep2 > external2;
        ev2= cs2-m2.GetVelocity();
        m2.SetTorque((ev2 > internal2));

        ep3=posan3-m3.GetPosition();
        cs3=ep3 > external3;
        ev3= cs3-m3.GetVelocity();
        m3.SetTorque((ev3 > internal3));

        cout << "ACTUAL: , " << m1.GetPosition() << " , " << m2.GetPosition() <<  " , " << m3.GetPosition() <<endl<<endl;
        //graph << t << " , " << posan1 << " , " << m1.GetPosition() << " , " << posan2 << " , " << m2.GetPosition()  << " , " << posan3 <<  " , " << m3.GetPosition()  << " , " << incli << " , " << incSensor   << " , " << orient   << " , " << oriSensor <<endl;

        tools.WaitSamplingTime();
    }
    sleep(1);

    cout << "FIN" << endl;
    m1.SetTorque(0);
    m2.SetTorque(0);
    m3.SetTorque(0);

    return 0;

}
