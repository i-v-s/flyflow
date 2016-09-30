#include <gtest/gtest.h>
#include <string>

enum Enum
{
    Height,
    Attitude,
    Velocity,
    Position
};

#include "vectpl.h"

TEST(vtpVectorTest, basic)
{
    vtp::Vector<vtp::Tag<Velocity, double>, vtp::Tag<Position, int>, vtp::Tag<Attitude, std::string>> v1;
    EXPECT_TRUE(v1.has<Attitude>());
    vtp::Vector<vtp::Tag<Velocity, double>, vtp::Tag<Attitude, std::string>> v2;
    vtp::Vector<vtp::Tag<Position, int>, vtp::Tag<Attitude, std::string>, vtp::Tag<Velocity, double>> v3;
    vtp::get<Velocity>(v1) = 1.4;
    vtp::get<Position>(v1) = 1;
    vtp::get<Velocity>(v2) = 1.4;
    vtp::get<Attitude>(v2) = "test";
    vtp::get<Position>(v3) = 2;
    vtp::get<Attitude>(v3) = "trust";

    static_assert(v2.subsetOf(v1), "error");
    static_assert(!v1.subsetOf(v2), "error");
    v1 += v2;
    v3 += v2;
    EXPECT_EQ(vtp::get<Velocity>(v1), 2.8);
    EXPECT_EQ(vtp::get<Position>(v1), 1);

    EXPECT_EQ(vtp::get<Attitude>(v3), "trusttest");
    EXPECT_EQ(vtp::get<Position>(v3), 2);
}

TEST(vtpVectorTest, sum)
{
    using namespace vtp;
    typedef Vector<Tag<Velocity, double>, Tag<Height, double>> V1;
    typedef Vector<Tag<Position, double>, Tag<Height, double>> V2;
    //typedef typename VectorUnion<V1, V2>::Result V3;
    V1 v1;
    get<Velocity>(v1) = 1.1;
    get<Height>(v1) = 1.3;
    V2 v2;
    get<Position>(v2) = 1.4;
    get<Height>(v2) = 1.7;
    auto v3 = v1 + v2;
    EXPECT_EQ(get<Velocity>(v3), 1.1);
    EXPECT_EQ(get<Position>(v3), 1.4);
    EXPECT_EQ(get<Height>(v3), 3.0);
}

TEST(vtpVectorTest, vectorUnion)
{
    using namespace vtp;
    typedef Vector<vtp::Tag<Velocity, double>> V1;
    static_assert(V1::template subsetOf<V1>(), "subset error");
    typedef Vector<vtp::Tag<Attitude, std::string>> V2;
    static_assert(!V1::template subsetOf<V2>(), "subset error");
    typedef VectorUnion<V1, V2> V3;
    static_assert(V1::template subsetOf<V3>(), "subset error");
    static_assert(V2::template subsetOf<V3>(), "subset error");
    static_assert(!V3::template subsetOf<V1>(), "subset error");
}

TEST(vtpMatrixTest, basic)
{
    using namespace vtp;
    Matrix<
            Tag<Velocity, Vector<Tag<Velocity, double>, Tag<Position, double>>>,
            Tag<Position, Vector<Tag<Velocity, double>, Tag<Position, double>>>
            > mat1;
    Matrix<
            Tag<Velocity, Vector<Tag<Velocity, double>, Tag<Position, double>>>,
            Tag<Position, Vector<Tag<Velocity, double>, Tag<Position, double>>>
            > mat2;
    auto mat3 = mat1 * mat2;
}
