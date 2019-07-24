
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
    IPlot plotPos(dts);

    PIDBlock pid(0.5*1.5077,0.75378,0,dts);


    //    Motor setup
    m31.Reset();
    m31.SwitchOn();

    m31.Setup_Velocity_Mode(0,1);

    double target_position = 1;
    double cs = 0; //control signal
    double p = 0;


    SamplingTime tools;
    tools.SetSamplingTime(dts);


    for(double t=0; t<5; t+=dts){


        p = m31.GetPosition();
        cs = (target_position - p) > pid;
        m31.SetVelocity(cs);

        plotPos.pushBack(p);

        cout<<"t: "<<t<<", p: "<<p<<endl;
        //cout<<"cs: "<<cs<<endl;
        tools.WaitSamplingTime();

    }
    m31.SetVelocity(0);
    sleep(1);

    plotPos.Plot();

}

