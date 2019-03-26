#include "Cia402device.h"
#include "CiA301CommPort.h"
#include "SocketCanPort.h"
#include <iostream>


int main ()
{
    //--Can port communications--
    SocketCanPort pm1("can1");
    SocketCanPort pm2("can1");
    SocketCanPort pm3("can1");

    CiA402SetupData sd1(4096,3.7,0.001,4);
    CiA402SetupData sd2 = sd1;
    CiA402SetupData sd3 = sd1;

    CiA402Device m1 (1, &pm1, sd1);
    CiA402Device m2 (2, &pm2, sd2);
    CiA402Device m3 (3, &pm3, sd3);



//    Motor setup
     m1.Reset();
     m1.SwitchOn();

     m2.Reset();
     m2.SwitchOn();

     m3.Reset();
     m3.SwitchOn();


//     m3.Setup_Torque_Mode();
//     m1.Setup_Torque_Mode();
//     m2.Setup_Torque_Mode();

     m1.SetupPositionMode(1,1);
     m2.SetupPositionMode(1,1);
     m3.SetupPositionMode(1,1);

     cout << "Motor Start" <<endl;


}
