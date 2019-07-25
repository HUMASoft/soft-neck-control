

#include <iostream>
#include <fstream>
#include "Cia402device.h"
#include "CiA301CommPort.h"
#include "SocketCanPort.h"
#include "math.h"

#include "SerialArduino.h"


#include "fcontrol.h"
#include "IPlot.h"
#include "OnlineSystemIdentification.h"

#include "Kinematics.h"



int main ()
{
    //--sensor--
    SerialArduino tilt;
    float incSensor,oriSensor;
//    sleep(4); //wait for sensor


    //Samplinfg time
    double dts=0.02;
    SamplingTime Ts(dts);


    //tau = 0.1
//    0.09516
//   ----------
//   z - 0.9048
    SystemBlock filter(0.09516,0,- 0.9048,1);

    int numOrder=0,denOrder=2;
    OnlineSystemIdentification model(numOrder,denOrder,filter);
    SystemBlock sys;
//    FPDBlock con;
    PIDBlock con(1,0.01,0,dts);


    FPDTuner tuner(60,2);



//    model.SetFilter(filter);

    string folder="../";
    string mass="300g";

    ofstream file(folder+mass+".csv");



    //m1 setup
    SocketCanPort pm31("can1");
    CiA402SetupData sd31(2048,24,0.001, 0.144);
    CiA402Device m1 (31, &pm31, &sd31);
    m1.Reset();
    m1.SwitchOn();
    m1.SetupPositionMode();
//    m1.Setup_Velocity_Mode(5);


    //m2
    SocketCanPort pm2("can1");
    CiA402SetupData sd32(2048,24,0.001, 0.144);
    CiA402Device m2 (32, &pm2, &sd32);
    m2.Reset();
    m2.SwitchOn();
    m2.SetupPositionMode();
//    m2.Setup_Velocity_Mode(5);

    //m3
    SocketCanPort pm3("can1");
    CiA402SetupData sd33(2048,24,0.001, 0.144);
    CiA402Device m3 (33, &pm3, &sd33);
    m3.Reset();
    m3.SwitchOn();
    m3.SetupPositionMode();
//    m3.Setup_Velocity_Mode(5);



    //controllers
//    PIDBlock c1(2,1,0,dts);
    FPDBlock c1(8.67,20.53,-0.83,dts);
    FPDBlock c2(8.67,20.53,-0.83,dts);
    FPDBlock c3(8.67,20.53,-0.83,dts);



    double ep1,ev1,cs1;
    double tp1,tv1,v1,p1;
    double ep2,ev2,cs2;
    double tp2,tv2,v2,p2;
    double ep3,ev3,cs3;
    double tp3,tv3,v3,p3;

    double ierror, cs;



    IPlot plot1,plot2,plot3,id;


    //--Neck Kinematics--
    double l0=0.1085;
    double lg0=l0+0.002;
    double radio=0.0075; //winch radius
    GeoInkinematics neck_ik(0.052,0.052,l0); //kinematics geometric
    vector<double> lengths(3);

    double inc=20.0; //inclination tendon length
    double ori=180*M_PI/180; //target orientation
    double da2=2*M_PI/3, da3=4*M_PI/3; //angle shift for tendons 2 and 3

    for (double t=0; t<6; t+=dts)
    {
    if (tilt.readSensor(incSensor,oriSensor)>=0) break;
    }

//    neck_ik.GetIK(inc,90,lengths);
//    tp1=(lg0-lengths[0])/radio;
//    tp2=(lg0-lengths[1])/radio;
//    tp3=(lg0-lengths[2])/radio;

    cout << "tp1 " << tp1 << ", tp2 " << tp2 << ", tp3 " << tp3 <<endl;

    file << "time,tp1,p1,cs1,tp2,p2,cs2,tp3,p3,cs3" << endl;


//    vector<double> inc(interval/dts);
//    for i

    double interval=8; //in seconds
    for (double t=0;t<interval; t+=dts)
    {

        if (tilt.readSensor(incSensor,oriSensor) <0)
        {
            cout << "Sensor error! " << endl;
            //Due to sensor error set motors zero velocity.
//            m1.SetVelocity(0);
//            m2.SetVelocity(0);
//            m3.SetVelocity(0);

        }

        //target
//        inc=20/*-5*t*/+0.1*((rand() % 10 + 1)-5);

//        cout << "tp1 " << tp1 << ", tp2 " << tp2 << ", tp3 " << tp3 <<endl;
//        cout << "incli_sen: " << incSensor << " , orient_sen: " << oriSensor << endl;
        model.UpdateSystem(inc,incSensor);

        model.GetSystemBlock(sys);

//        tuner.TuneIsom(sys,con);

        //negative feedback
        ierror = inc - incSensor;



//        cout << "ierror " <<  ierror  << ", cs " << cs << ", incSensor " << incSensor <<endl;





//velocity strategy (activate also SetupVelocityMode())
/*

        //controller computes control signal
        cs = ierror/1000 > con;

        cs1=(cs*cos(ori))/radio;
        cs2=(cs*cos(ori+da2))/radio;
        cs3=(cs*cos(ori+da3))/radio;
        m1.SetVelocity(cs1);
        m2.SetVelocity(cs2);
        m3.SetVelocity(cs3);
        cout << "cs1 " << cs1 << ", cs2 " << cs2 << ", cs3 " << cs3 <<endl;

        */
//velocity strategy (activate also SetupVelocityMode())




//Position Strategy (activate also SetupPositionMode())

        //controlled inclination (cs)
        neck_ik.GetIK(cs,ori,lengths);
        tp1=(lg0-lengths[0])/radio;
        tp2=(lg0-lengths[1])/radio;
        tp3=(lg0-lengths[2])/radio;
        m1.SetPosition(tp1);
        m2.SetPosition(tp2);
        m3.SetPosition(tp3);

        cout << "tp1 " << tp1 << ", tp2 " << tp2 << ", tp3 " << tp3 <<endl;

//Position Strategy (activate also SetupPositionMode())

/* Velocity local loops
        file << t << ",";

        //loopm1
        ep1=tp1- m1.GetPosition();
        cs1= ep1 > c1;
        m1.SetVelocity(cs1);
        p1=m1.GetPosition();
        plot1.pushBack(p1);

        file << tp1 << "," << p1 << ","<< cs1 << ",";

        //loopm2
        ep2=tp2- m2.GetPosition();
        cs2= ep2 > c2;
        m2.SetVelocity(cs2);
        p2=m2.GetPosition();
        plot2.pushBack(p2);

        file << tp2 << ","<< p2 << ","<< cs2 << ",";


        //loopm3
        ep3=tp3- m3.GetPosition();
        cs3= ep3 > c3;
        m3.SetVelocity(cs3);
        p3=m3.GetPosition();
        plot3.pushBack(p3);

        file << tp3 << ","<< p3 << ","<< cs3  << endl;

*/

        Ts.WaitSamplingTime();


    }




    model.PrintZTransferFunction(dts);
    double phi,mag,w=2;
    model.GetMagnitudeAndPhase(dts,w,mag,phi);

    cout << "mag: " << mag  << "phi: " << phi << endl ;

    m1.SetVelocity(0);
    m2.SetVelocity(0);
    m3.SetVelocity(0);



//    id.Plot();
//    plot1.Plot();
//    plot2.Plot();
//    plot3.Plot();

//    p1.PlotAndSave("../pos.csv");


//    m1.SetupPositionMode(1);
//    m2.SetupPositionMode(1);
//    m3.SetupPositionMode(1);
//    sleep(1);
//    m1.SetPosition(0);
//    m2.SetPosition(0);
//    m3.SetPosition(0);

    sleep(2);

    m1.SwitchOff();
    m2.SwitchOff();
    m3.SwitchOff();



file.close();

return 0;

}

