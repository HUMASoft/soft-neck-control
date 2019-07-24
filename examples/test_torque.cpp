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

    CiA402Device m31 (31, &pm31, &sd31);

    double dts=0.001;
    double v;

    SamplingTime tools;
    tools.SetSamplingTime(dts);


    //    Motor setup
    m31.Reset();
    m31.SwitchOn();

    m31.Setup_Torque_Mode();
    //m31.Setup_Velocity_Mode(0,1);


    m31.SetTorque(0.8);
    //m31.SetVelocity(5);

    for(double t=0; t<2; t+=dts){
        v= m31.GetVelocity();
        cout <<"vel: "<< v <<endl;
        tools.WaitSamplingTime();
    }


    sleep(1);
    m31.Setup_Velocity_Mode(0,1);
    m31.SetVelocity(0);


}
