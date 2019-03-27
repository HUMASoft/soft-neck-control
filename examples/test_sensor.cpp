#include "Cia402device.h"

#include "SerialArduino.h"
#include <iostream>
#include "ToolsFControl.h"

int main(){
    //--sensor--
    SerialArduino tilt;
    float incSensor,oriSensor;
    double dts=0.01;
    sleep(4); //wait for sensor

    ToolsFControl tools;
    tools.SetSamplingTime(dts);

    for (double t=0;t<1000;t+=dts){

        tilt.readSensor(incSensor,oriSensor);
        cout << "incli_sen: " << incSensor << " , orient_sen: " << oriSensor << endl;
        tools.WaitSamplingTime();
    }

}
