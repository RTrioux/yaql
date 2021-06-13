#include <iostream>
#include <math.h>
#include "quat.hpp"
using namespace std;

int main()
{
    bool extrinsic = true;
    Quat q1 = Quat::unitQuat(0.3,70,-20,33);
    Quat::Sequence seq = Quat::XYX;

    Vector3D euler = Vector3D(q1.toEuler(seq,false,extrinsic));
    Quat qFromEuler = Quat::fromEuler(euler,seq,false,extrinsic);
    Vector3D toEuler = Vector3D(qFromEuler.toEuler(seq,false,extrinsic)); 

    cout<<q1<<endl;
    cout<<euler<<endl;
    cout<<qFromEuler<<endl;
    cout<<toEuler<<endl;

    return 0;
}