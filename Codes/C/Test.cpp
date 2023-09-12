#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <vector>
#include <unistd.h>
#include <termio.h>

using namespace std;

template<typename T>
T func(T a,T b)
{
    return a + b;
}


int main()
{
    cout << func(123,135);
    return 0;
}