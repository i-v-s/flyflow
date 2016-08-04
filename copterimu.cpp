#include "copterimu.h"
#include "timefusion.h"
#include <gtest/gtest.h>

TEST(CopterImuTest, basic)
{
    typedef double Time;
    typedef double Scalar;
    //ImuSequence<Time, Scalar> imuSeq;
    TimeFusion<Time, ImuSequence<Time, Scalar>> tf;
    typedef ImuPoint<Time, Scalar>::Measure Measure;
    Measure m;
    m.a << 0.0, 0.0, 9.8;
    m.w << 0.0, 0.0, 0.0;
    tf.onMeasure(0.0, m);


}
