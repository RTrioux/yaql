#include <iostream>
#include <math.h>
#include "quat.hpp"
using namespace std;

int main()
{
    Quat q1 = Quat::unitQuat(M_PI_4,1,2,3);
    cout<<q1<<endl;
    cout<<Vector3D(q1.toEuler(Quat::XZX))<<endl;
    cout<<Quat::fromEuler(q1.toEuler(Quat::XZX) ,Quat::XZX)<<endl;
    cout<<Vector3D(Quat::fromEuler(q1.toEuler(Quat::XZX) ,Quat::XZX).toEuler(Quat::XZX))<<endl;
    return 0;
}