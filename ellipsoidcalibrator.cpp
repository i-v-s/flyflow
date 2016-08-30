#include "ellipsoidcalibrator.h"
#include <gtest/gtest.h>

TEST(EllipsoidCalibratorTest, basic)
{
    EllipsoidCalibrator<double> ec;
    double vals[][3] = {
        { 0, 0, 0},
        {10, 0, 0},
        { 5, 2, 0},
        { 5,-2, 0},
        { 5, 0, 3},
        { 5, 0,-3}
    };
    for(int t = 0; t < sizeof(vals) / sizeof(vals[0]); t++)
        ec.addValue(vals[t]);
    double offset[3], scale[3];
    ec.solve(offset, scale);
    EXPECT_NEAR(offset[0], 5, 1E-10);
    EXPECT_NEAR(offset[1], 0, 1E-10);
    EXPECT_NEAR(offset[2], 0, 1E-10);
    EXPECT_NEAR(scale[0], 5, 1E-10);
    EXPECT_NEAR(scale[1], 2, 1E-10);
    EXPECT_NEAR(scale[2], 3, 1E-10);
}
