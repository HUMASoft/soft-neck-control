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


cout<<"torque 0"<<endl;
    m31.SetTorque(0);

    sleep(5);

    cout<<m31.GetVelocity()<<endl;

//    double dts=0.01;

//    ToolsFControl tools;
//    tools.SetSamplingTime(dts);
//    //
//cout<<"torque 0.5"<<endl;

//    m31.SetTorque(-1);

//    for(double t=0; t<2; t+=dts){
//         cout<<m31.GetVelocity()<<endl;
//         tools.WaitSamplingTime();

//    }

//    cout<<"torque 0"<<endl;
//   m31.SetTorque(0.0);
//      m31.SetTorque(0.0);
//         m31.SetTorque(0.0);
//   sleep(1);

}
