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
#include "tuples.h"

TEST(vtpVectorTest, basic)
{
    typedef vtp::Vector<vtp::Tag<Velocity, double>, vtp::Tag<Position, int>, vtp::Tag<Attitude, std::string>> V1;
    static_assert(V1::size() == 3, "Vector::size() error");
    static_assert(V1::has<Attitude>(), "Vector::has() error");
    static_assert(!V1::has<Height>(), "Vector::has() error");
    typedef vtp::Reverse<V1> RV1;
    static_assert(RV1::size() == 3, "Vector::size() error");
    static_assert(RV1::has<Attitude>(), "Vector::has() error");
    static_assert(!RV1::has<Height>(), "Vector::has() error");

    V1 v1;
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
    typedef Vector<Tag<Height, double>, Tag<Position, double>> V2;
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
    Vector<Tag<Velocity, double>, Tag<Height, double>> v4;
    get<Velocity>(v4) = 1.0;
    get<Height>(v4) = 2.0;
    Vector<Tag<Height, double>, Tag<Velocity, double>> v5;
    get<Height>(v5) = 8.0;
    get<Velocity>(v5) = 4.0;
    auto v6 = v4 + v5;
    EXPECT_EQ(get<Velocity>(v6), 5.0);
    EXPECT_EQ(get<Height>(v6), 10.0);
}

TEST(vtpVectorTest, vectorUnion)
{
    using namespace vtp;
    typedef Vector<vtp::Tag<Velocity, double>> V1;
    static_assert(V1::size() == 1, "Vector::size() error");
    static_assert(V1::template subsetOf<V1>(), "subset error");
    typedef Vector<vtp::Tag<Attitude, std::string>> V2;
    static_assert(V1::size() == 1, "Vector::size() error");
    static_assert(!V1::template subsetOf<V2>(), "subset error");
    typedef typename UnionT<V1, V2>::Result V3;
    static_assert(V3::has<Velocity>(), "Union<> error");
    static_assert(V3::has<Attitude>(), "Union<> error");
    static_assert(V1::template subsetOf<V3>(), "subset error");
    static_assert(V2::template subsetOf<V3>(), "subset error");
    static_assert(!V3::template subsetOf<V1>(), "subset error");
    static_assert(V3::size() == 2, "Vector::size() error");
    typedef Union<V1, V3> V4;
    static_assert(V4::size() == 2, "Vector::size() error");

    typedef Union<V3, V1> V5;
    static_assert(V5::size() == 2, "Vector::size() error");
}

TEST(vtpVectorTest, matrixUnion)
{
    using namespace vtp;
    typedef Matrix<
            Tag<Velocity, Vector<Tag<Velocity, double>, Tag<Position, double>>>,
            Tag<Position, Vector<Tag<Velocity, double>>>
    > M1;
    typedef Matrix<
            Tag<Velocity, Vector<Tag<Velocity, double>, Tag<Position, double>>>,
            Tag<Position, Vector<Tag<Velocity, double>, Tag<Position, double>>>
    > M2;

    typedef UnionT<M1, M2>::Result M3;
    static_assert(M1::template subsetOf<M3>(), "subset error");
    static_assert(M2::template subsetOf<M3>(), "subset error");
    static_assert(M3::template subsetOf<M2>(), "subset error");
    static_assert(M3::template subsetOf<M1>(), "subset error");
    typedef typename FindItemT<Position, typename M3::Parent>::Item M3Pos;
    static_assert(M3Pos::has<Velocity>(), "M3Pos");
    static_assert(M3Pos::has<Position>(), "M3Pos");
    static_assert(!M3Pos::has<Height>(), "M3Pos");

    //typedef Vector
}

TEST(vtpMatrixTest, basic)
{
    using namespace vtp;
    typedef Matrix<
            Tag<Velocity, Vector<Tag<Velocity, double>, Tag<Position, double>>>,
            Tag<Position, Vector<Tag<Velocity, double>>>
            > Mat;
    Mat mat;
    get<Velocity, Position>(mat) = 2.0;
    get<Position, Velocity>(mat) = 4.0;
    const auto &cmat = mat;
    auto pv = get<Position, Velocity>(cmat);
    auto vp = get<Velocity, Position>(cmat);
    EXPECT_EQ(pv, 4.0);
    EXPECT_EQ(vp, 2.0);

    typedef Matrix<
            Tag<Velocity, Vector<Tag<Velocity, double>, Tag<Position, double>>>,
            Tag<Position, Vector<Tag<Velocity, double>, Tag<Position, double>>>
            > Mat2;
    Mat2 mat2;
    get<Velocity, Position>(mat2) = 20.0;
    get<Position, Velocity>(mat2) = 40.0;
    get<Position, Position>(mat2) = 70.0;
    auto mat3 = cmat + mat2;
    auto &matp = get<Position>((const typename Mat::Parent &) mat);
    auto &mat2p = get<Position>((const typename Mat2::Parent &) mat2);
    pv = get<Position, Velocity>(mat3);
    vp = get<Velocity, Position>(mat3);
    auto pp = get<Position, Position>(mat2);
    EXPECT_EQ(pv, 44.0);
    EXPECT_EQ(vp, 22.0);
    EXPECT_EQ(pp, 70.0);
}

TEST(vtpMatrix, makeTranspose)
{
    using namespace vtp;
    typedef Matrix<
        Tag<Velocity, Vector<                       Tag<Velocity, double>, Tag<Position, double>>>,
        Tag<Position, Vector<Tag<Attitude, double>,                        Tag<Position, double>>>
    > Mat;
    typedef FindItem(Velocity, Mat::Vectors) VelVecHor;
    typedef MakeVectorTranspose<VelVecHor, Velocity> VelVecVer;
    static_assert(VelVecVer::has<Velocity>(), "Matrix::has() error");
    typedef FindItem(Position, VelVecVer) PVV;
    static_assert(PVV::has<Velocity>(), "Matrix::has() error");
    static_assert(VelVecVer::has<Position>(), "Matrix::has() error");
    static_assert(!VelVecVer::has<Attitude>(), "Matrix::has() error");
    static_assert(!VelVecVer::has<Height>(), "Matrix::has() error");

    typedef FindItem(Position, Mat::Parent) PosVecHor;
    typedef MakeVectorTranspose<PosVecHor, Position> PosVecVer;
    typedef typename UnionT<VelVecVer, PosVecVer>::Result Tr;
    typedef typename VectorMergeLeft<Vector<>, VelVecVer, PosVecVer>::Result VML;

    typedef typename UnionT<VelVecVer, PosVecVer>::Result R1;
    typedef typename VectorAbsentAdd<VML, PosVecVer>::Result R;

    typedef MatrixTranspose<Mat> TMat;
    //TMat::test;
    static_assert(TMat::has<Attitude>(), "Matrix::has() error");
    static_assert(TMat::has<Velocity>(), "Matrix::has() error");
    static_assert(TMat::has<Position>(), "Matrix::has() error");
}

TEST(vtpMatrix, mul)
{
    using namespace vtp;
    using V1 = Vector<Tag<Velocity, int>, Tag<Attitude, double>>;
    using V2 = Vector<Tag<Velocity, int>, Tag<Height, double>>;
    VecVecMul<V1, V2> t;

    //t.rt;

    typedef Matrix<
            Tag<Velocity, Vector<Tag<Velocity, double>, Tag<Position, double>>>,
            Tag<Position, Vector<Tag<Velocity, double>, Tag<Position, double>>>
            > Mat2;
    /*typedef MatrixMul<Mat2, Mat2> M;
    Mat2 mat2;
    auto mat3 = mat2 * mat2;*/
}
