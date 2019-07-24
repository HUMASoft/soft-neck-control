
#include "Cia402device.h"
#include "CiA301CommPort.h"
#include "SocketCanPort.h"
#include <iostream>

#include "fcontrol.h"
#include "IPlot.h"


int main ()
{
    //--Can port communications--
    SocketCanPort pm31("can1");

    CiA402SetupData sd31(2048,24,0.001, 0.144);

    CiA402Device m31 (31, &pm31, &sd31);



    double dts=0.001;
    IPlot plotVel(dts);

    PIDBlock pid(1.5077,0.75378,0,dts);


    //    Motor setup
    m31.Reset();
    m31.SwitchOn();

    m31.Setup_Torque_Mode();

    double tspeed = 2;
    double cs = 0; //control signal
    double v = 0;


    SamplingTime tools;
    tools.SetSamplingTime(dts);

    for(double t=0; t<5; t+=dts){

        v = m31.GetVelocity();
        cs = (tspeed - v) > pid;
        cs=cs/10000;
        m31.SetTorque(cs); // tanto por 1
        cout<<"t: "<<t<<", v: "<<v<<endl;

        cout<<"cs: "<<cs<<endl;

        plotVel.pushBack(v);
        tools.WaitSamplingTime();

    }
    //
    m31.SetTorque(0);
    m31.Setup_Velocity_Mode(0,1);
    m31.SetVelocity(0);
    sleep(1);

    plotVel.Plot();



}
