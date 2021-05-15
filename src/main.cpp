#include <iostream>
#include "quat.hpp"
using namespace std;

int main()
{
    Quat q1(0.2,2,3,7);
    Quat q2(0.5,11,13,17); 
    Quat q3 = q1+q2;
    
    cout<<q3<<endl;
    cout<<q1.re()<<endl;
    return 0;
}