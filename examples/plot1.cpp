
#include <complex>

#include "fcontrol.h"
#include <math.h>

#include "IPlot.h"

using namespace std;

int main()
{


    double dts=0.01;
    IPlot p(dts);

    for (double t=0; t<2; t+=0.01)

    {
        p.pushBack(t*0.1);

    }


    p.Plot();

    return 0;
}
