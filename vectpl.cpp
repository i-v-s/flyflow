#include <gtest/gtest.h>
#include <string>

enum Enum
{
    None,
    Attitude,
    Velocity,
    Position
};

#include "vectpl.h"

TEST(vtpVectorTest, basic)
{
    vtp::Vector<vtp::Tag<Velocity, double>, vtp::Tag<Position, int>> v1;
    vtp::Vector<vtp::Tag<Velocity, double>, vtp::Tag<Attitude, std::string>> v2;
    vtp::Vector<vtp::Tag<Position, int>, vtp::Tag<Attitude, std::string>> v3;
    vtp::get<Velocity>(v1) = 1.4;
    vtp::get<Position>(v1) = 1;
    vtp::get<Velocity>(v2) = 1.4;
    vtp::get<Attitude>(v2) = "test";
    vtp::get<Position>(v3) = 2;
    vtp::get<Attitude>(v3) = "trust";

    v1 += v2;
    v3 += v2;
    EXPECT_EQ(vtp::get<Velocity>(v1), 2.8);
    EXPECT_EQ(vtp::get<Position>(v1), 1);

    EXPECT_EQ(vtp::get<Attitude>(v3), "trusttest");
    EXPECT_EQ(vtp::get<Position>(v3), 2);
}
