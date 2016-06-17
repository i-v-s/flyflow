#include "sequencestd.h"
#include <gtest/gtest.h>


TEST(SequenceTest, basic)
{
    struct Data
    {
        int a, b;

    };
    typedef double Time;
    Stamped<Time, Data> st(0.0, Data());
    SequenceDeque<Time, Data> seq;
    seq.add(0.0, Data());

    //seq.insert(std::make_pair<Time, Data>(1.0, { 1, 2 }));
    //EXPECT_TRUE(seq.findBefore(0.5) == seq.end());
    //EXPECT_EQ(seq.findBefore(0.5), seq.end());
}
