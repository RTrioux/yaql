#include <iostream>
#include <math.h>
#include "quat.hpp"
using namespace std;

int main()
{
    Quat q1 = Quat::unitQuat(M_PI_2,1,1,0);
    Quat::Sequence seq = Quat::XYZ;

    Vector3D euler = Vector3D(q1.toEuler(seq));
    Quat qFromEuler = Quat::fromEuler(euler,seq);
    Vector3D toEuler = Vector3D(qFromEuler.toEuler(seq)); 

    cout<<q1<<endl;
    cout<<euler<<endl;
    cout<<qFromEuler<<endl;
    cout<<toEuler<<endl<<endl;

    //cout<<Quat::fromEuler(-0.92752418,0.15945954,1.28677213,seq)<<endl;

    cout<<q1.rotateVector(Vector3D(0,1,0))<<endl;
    cout<<qFromEuler.rotateVector(Vector3D(0,1,0))<<endl;
    return 0;
}