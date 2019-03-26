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

double pos1, pos2, pos3;
m1.SetPosition(2.5);

for (int i=0;i<10;i++){
 pos1 = m1.GetPosition();
 cout<<pos1<<endl;
 sleep(1);
}

}
