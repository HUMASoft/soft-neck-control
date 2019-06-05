#include "Cia402device.h"
#include "CiA301CommPort.h"
#include "SocketCanPort.h"
#include <iostream>
#include "IPlot.h"

using namespace std;

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

    // motors must be turn ON

    double pos;
    double vel;


    double dts=0.01;
    IPlot p(dts),v(dts);


    //    Motor setup
    m31.Reset();
    m31.SwitchOn();

    //set velocity and aceleration (rads/s)
    m31.SetupPositionMode(1,1);


    // position  [rads]
    m31.SetPosition(1);


    for (int i=0;i<50;i++){

        pos = m31.GetPosition();

        vel = m31.GetVelocity();

        p.pushBack(pos);
        v.pushBack(vel);


        cout<<"pos1: "<<pos<<endl;
        cout<<"vel1: "<<vel<<endl;
        usleep(1000*100);
    }

    v.Plot();
    p.Plot();

    // position  [rads]
    m31.SetPosition(0);
    sleep(3);

}
