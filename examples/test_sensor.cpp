#include "Cia402device.h"

#include "SerialArduino.h"
#include <iostream>
#include "ToolsFControl.h"
#include "SystemBlock.h"

int main(){
    //--sensor--
    SerialArduino tilt;
    float incSensor,oriSensor;
    double dts=0.01;
    sleep(4); //wait for sensor
    SystemBlock filterSensor(0.09516,0,- 0.9048,1);


    SamplingTime tools;
    tools.SetSamplingTime(dts);

    for (double t=0; t<6; t+=dts)
    {
        tools.WaitSamplingTime();
        if (tilt.readSensor(incSensor,oriSensor)>=0)
        {
            cout << "Starting sensor " << endl;
            break;
        }
    }
    cout << "Sensor started" << endl;


    for (double t=0;t<1000;t+=dts){

        if (tilt.estimateSensor(incSensor,oriSensor)<0)
//        if (tilt.readSensor(incSensor,oriSensor)<0)
        {
            cout << "Sensor read error !" << endl;
        }
        else
        {
            cout << "incli_sen: " <<  (incSensor > filterSensor) << " , orient_sen: " << oriSensor << endl;
        }
        cout << "Available time: " << tools.WaitSamplingTime() << endl;
    }

}
