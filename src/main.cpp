#include <iostream>
#include <math.h>
#include "quat.hpp"
using namespace std;

int main()
{
    Quat q1 = Quat::unitQuat(M_PI_4,0,1,0);
    cout<<q1.rotateVector(Vector3D(1,0,0))<<endl;
    cout<<q1<<endl;
    cout<<q1.re()<<endl;
    return 0;
}