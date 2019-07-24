
#include "Cia402device.h"
#include "CiA301CommPort.h"
#include "SocketCanPort.h"
#include <iostream>

#include "fcontrol.h"


int main ()
{
    //--Can port communications--
    SocketCanPort pm31("can1");
    SocketCanPort pm32("can1");
    SocketCanPort pm33("can1");

    CiA402SetupData sd31(2048,24,0.001, 0.144);
    CiA402SetupData sd32(2048,24,0.001, 0.144);
    CiA402SetupData sd33(2048,24,0.001, 0.144);

    CiA402Device m31 (31, &pm31, &sd31);
//    CiA402Device m32 (32, &pm32, &sd32);
//    CiA402Device m33 (33, &pm33, &sd33);

    double enc1,enc2,enc3;
    double dts=0.001;
    SamplingTime tools;
    tools.SetSamplingTime(dts);

    m31.Setup_Velocity_Mode(0,1);
    m31.SetVelocity(0);

    for(double t=0; t<15; t+=dts){

        m31.SetVelocity(0);

        enc1 = m31.GetPosition();
//        enc2 = m32.GetPosition();
//        enc3 = m33.GetPosition();

        cout<<"Motor31: "<<enc1<<"  Motor32: "<<enc2<<" Motor33: "<<enc3<<endl;

        tools.WaitSamplingTime();
    }


}
