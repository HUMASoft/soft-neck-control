#include "Cia402device.h"
#include "CiA301CommPort.h"
#include "SocketCanPort.h"
#include <iostream>


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
    CiA402Device m32 (32, &pm32, &sd32);
    CiA402Device m33 (33, &pm33, &sd33);


//    Motor setup
     m31.Reset();
     m31.SwitchOn();

     m32.Reset();
     m32.SwitchOn();

     m33.Reset();
     m33.SwitchOn();
//     m33.DisablePDOs();


    //set velocity and aceleration (rads/s)
//     m31.SetupPositionMode(1,1);
//     m32.SetupPositionMode(1,1);
//     m33.SetupPositionMode(1,1);

     cout << "Motors Started" <<endl;


}
