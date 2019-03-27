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

    // motors must be turn ON

    double pos1, pos2, pos3;
    double vel1, vel2, vel3;

    // position  [rads]
    m1.SetPosition(2.5);
    m2.SetPosition(1.5);
    m3.SetPosition(3.2);

    for (int i=0;i<10;i++){

        pos1 = m1.GetPosition();
        pos2 = m2.GetPosition();
        pos3 = m3.GetPosition();

        vel1 = m1.GetVelocity();
        vel2 = m2.GetVelocity();
        vel3 = m3.GetVelocity();

        cout<<"pos1: "<<pos1<<", "<<"pos2: "<<pos2<<", "<<"pos3: "<<pos3<<endl;
        cout<<"vel1: "<<vel1<<", "<<"vel2: "<<vel2<<", "<<"vel3: "<<vel3<<endl;
        sleep(1);
    }

}
