#include <iostream>
#include <gtest/gtest.h>
#include "quat.hpp"

int main()
{
    Quat q(1,2,3,4);
    std::cout<<q<<std::endl;
    return 0;
}
