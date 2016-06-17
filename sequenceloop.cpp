#include "sequenceloop.h"
#include <gtest/gtest.h>

struct Data
{
    double t;
    double time() { return t; }
    Data(double t) : t(t) { }
};


TEST(SequenceLoopTest, first_last)
{
    SequenceLoop<Data, 16> sl;
    EXPECT_FALSE(sl.last());
    EXPECT_FALSE(sl.first());
    auto t = sl.add(1.0);
    EXPECT_TRUE(t);
    EXPECT_TRUE(sl.last());
    EXPECT_TRUE(sl.first());
    EXPECT_EQ(sl.last(), t);
    EXPECT_EQ(sl.first(), sl.last());
    auto t2 = sl.add(2.0);
    EXPECT_EQ(sl.last(), t);
    EXPECT_EQ(sl.first(), t2);
}

TEST(SequenceLoopTest, findBeforeBasic)
{
    SequenceLoop<Data, 16> sl;
    EXPECT_EQ(sl.findBefore(1.0), sl.end());
    sl.add(1.0);
    EXPECT_EQ(sl.findBefore(0.5), sl.end());
    EXPECT_EQ(sl.findBefore(1.0), sl.end());
    EXPECT_NE(sl.findBefore(1.5), sl.end());
    sl.add(2.0);
    auto d1 = sl.findBefore(1.5);
    auto d2 = sl.findBefore(2.5);
    EXPECT_NE(d1, sl.end());
    EXPECT_NE(d2, sl.end());
    EXPECT_EQ(d1->time(), 1.0);
    EXPECT_EQ(d2->time(), 2.0);
}

TEST(SequenceLoopTest, findBefore)
{
    SequenceLoop<Data, 32> sl;
    for(int x = 0; x < 30; x++)
    {
        for(int y = 0; y <= x; y++)
        {
            Data * d = sl.findBefore(0.5 + y);
            if(0.5 + y < 1.0)
                EXPECT_EQ(d, sl.end());
            else
                EXPECT_NE(d, sl.end());
            if(d != sl.end())
                EXPECT_EQ(d->time(), y);
        }
        sl.add(1.0 + x);
    }
}

TEST(SequenceLoopTest, findBeforeLoop)
{
    SequenceLoop<Data, 32> sl;
    for(int x = 0; x < 100; x++)
    {
        for(int y = 0; y <= x; y++)
        {
            Data *last = sl.last();//, *first = sl.first();
            Data * d = sl.findBefore(0.5 + y);
            if(last && 0.5 + y > last->time())
                EXPECT_TRUE(d != sl.end());
            else
                EXPECT_FALSE(d != sl.end());
            if(d != sl.end())
                EXPECT_EQ(d->time(), y);
        }
        sl.add(1.0 + x);
    }
}

TEST(SequenceLoopTest, popDestruct)
{
    struct Data
    {
        char t;
        int &ctr;
        Data(int &ctr) : t(55), ctr(ctr) { ctr++; }
        ~Data()
        {
            EXPECT_EQ(t, 55);
            t = 0;
            ctr--;
            EXPECT_TRUE(ctr >= 0);
        }
    };
    int ctr = 0;
    SequenceLoop<Data, 32> sl;
    sl.add(ctr);
    EXPECT_EQ(ctr, 1);
    sl.add(ctr);
    EXPECT_EQ(ctr, 2);
    sl.drop();
    EXPECT_EQ(ctr, 1);
    sl.drop();
    EXPECT_EQ(ctr, 0);
    //ASSERT_DEATH({ sl.pop();}, "Assertion failed!");
    //EXPECT_EQ(ctr, 0);
}

TEST(SequenceLoopTest, destructor)
{
    struct Data
    {
        char t;
        int &ctr;
        Data(int &ctr) : t(55), ctr(ctr) { ctr++; }
        ~Data()
        {
            EXPECT_EQ(t, 55);
            t = 0;
            ctr--;
            EXPECT_TRUE(ctr >= 0);
        }
    };
    int ctr = 0;
    {
        SequenceLoop<Data, 32> sl;
        sl.add(ctr);
        sl.add(ctr);
        EXPECT_EQ(ctr, 2);
    }
    EXPECT_EQ(ctr, 0);
}
