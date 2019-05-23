
#include "Cia402device.h"
#include "CiA301CommPort.h"
#include "SocketCanPort.h"
#include <iostream>

#include "fcontrol.h"


int main ()
{
    //--Can port communications--
    SocketCanPort pm31("can1");

    CiA402SetupData sd31(2048,24,0.001, 0.144);

    CiA402Device m31 (31, &pm31, sd31);


    //    Motor setup
    m31.Reset();
    m31.SwitchOn();

    m31.Setup_Torque_Mode();

    double tspeed = 3;
    double cs = 0; //control signal
    double v = 0;

    //cout<<m31.GetVelocity()<<endl;

    double dts=0.001;

    ToolsFControl tools;
    tools.SetSamplingTime(dts);



    for(double t=0; t<5; t+=dts){


        v = m31.GetVelocity();
        cs = tspeed - v;
        m31.SetTorque(cs/10000);
        cout<<v<<endl;
        tools.WaitSamplingTime();
//        cout<<error<<endl;

    }

    //m31.SetTorque(0);

    m31.SetupPositionMode(1,1);
   sleep(1);

}
