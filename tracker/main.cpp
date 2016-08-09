#include <stdint.h>
//#include <iostream>
#include "../copterimu.h"
#include "../sequenceloop.h"
#include "../timefusion.h"

//using namespace std;
typedef uint32_t Time;
typedef uint32_t Scalar;
typedef ImuSequence<Time, Scalar, 128> ImuSeq;
typedef ImuPoint<Time, Scalar>::Measure Measure;

TimeFusion<Time, ImuSeq> fusion;

int main()
{
    Measure m;
    m.a << 0, 0, 0;
    fusion.onMeasure(0, m);
    //cout << "Hello World!" << endl;
    return 0;
}
