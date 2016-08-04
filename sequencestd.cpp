#include "sequencestd.h"
#include <gtest/gtest.h>


TEST(SequenceTest, basic)
{
    typedef double Time;
    struct Data : public Stamped<Time>
    {
        int a, b;
        Data(Time time) : Stamped<Time>(time) { }
    };
    //Stamped<Time, Data> st(0.0, Data());
    SequenceDeque<Data> seq;
    seq.add(0.0);

    //seq.insert(std::make_pair<Time, Data>(1.0, { 1, 2 }));
    //EXPECT_TRUE(seq.findBefore(0.5) == seq.end());
    //EXPECT_EQ(seq.findBefore(0.5), seq.end());
}
