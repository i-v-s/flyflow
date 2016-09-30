#include "imu.h"
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

#include <fstream>

/*Eigen::Quaterniond gravityToQuaternion(const Eigen::Vector3d &gravity)
{
    Eigen::Vector3d p;
    p << 0.0, 0.0, 1.0;
    p = p.cross(gravity);
    Eigen::Quaterniond q;

}*/

struct ImuTestResult
{


};

typedef double Time;

void testImuFromFile(const std::string &fn, Time t1b, Time t1e, Time t2b, Time t2e, double af, double wf)
{
    std::ifstream file(fn);
    std::string line;
    int p = 0;
    typedef double Scalar;
    typedef ImuPoint<Time, Scalar>::Measure Measure;
    Measure m;
    m.w.setZero();
    m.a.setZero();
    Time time, to = 0;
    Eigen::Vector3d a1, a2, w1, w2, v2, p2, wi;
    int c1 = 0, c2 = 0;
    Scalar l1, l2;
    a1.setZero();
    a2.setZero();
    w1.setZero();
    w2.setZero();
    wi.setZero();
    typedef ImuSequence<Time, Scalar, 1024> Seq;
    Seq seq;
    Seq::iterator it = nullptr;
    double gRes = 250.0 / 32768.0 * M_PI / 180;

    while(std::getline(file, line))
    {
        if(p > 0) for(size_t x = 0, xo = 0, i = 0; x < line.size(); x++) if(line[x] == ',')
        {
            int v = std::stoi(line.substr(xo, x - xo));
            switch(i)
            {
            case 0: time = v * 0.001; break;
            case 1: m.a[0] += ((v - 180) / 16400.0 * 9.8 - m.a[0]) * af; break;
            case 2: m.a[1] += ((v - 95) / 16400.0 * 9.8 - m.a[1]) * af; break;
            case 3: m.a[2] += ((v + 605) / 16535.0 * 9.8 - m.a[2]) * af; break;
            case 4: m.w[0] += ((v + 38)  * gRes - m.w[0]) * wf; break;
            case 5: m.w[1] += ((v - 91)  * gRes - m.w[1]) * wf; break;
            case 6: m.w[2] += ((v + 146) * gRes - m.w[2]) * wf; break;
            default: break;
            }

            xo = x + 1;
            i++;
        }
        p++;
        if(time > t1b && time < t1e) // Калибровка 1
        {
            a1 += m.a;
            w1 += m.w;
            c1++;
        }
        else if(c1)
        {
            a1 /= c1;
            w1 /= c1;
            c1 = 0;
            seq.setWBias(w1);
            l1 = a1.norm();
            seq.setGravity(l1);

        }
        if(time > t1e && time < t2b) // Интегрируем
        {
            if(to) wi += m.w * (time - to);
            to = time;
            Seq::iterator t = seq.add(time, m);
            if(it)
                t->from(std::make_pair(it, &seq), &seq);
            else
            {
                t->from(&seq);
                Eigen::Vector3d v0;
                v0 << 0, 0, 1;
                t->state.q.setFromTwoVectors(a1, v0);
            }
            it = t;
        }
        if(time > t2b && time < t2e) // Калибровка 2
        {
            a2 += m.a;
            w2 += m.w;
            c2++;
        }
        else if(c2)
        {
            a2 /= c2;
            w2 /= c2;
            l2 = a2.norm();
            c2 = 0;
        }
    }
    if(!it) return;
    v2 = it->state.v;
    p2 = it->state.p;
    Time dt = 11.85 - 10.33;
    p2 -= v2 * dt * 0.5;
    // Пересчёт
    Eigen::Vector3d v0;
    v0 << 0, 0, l2;
    //Eigen::Vector3d vt = it->state.q.inverse()._transformVector(v0);
    std::cout << "l1 << l2";

}

TEST(CopterImuTest, mpu9250)
{
    //Time t1b = 9.0, t1e = 9.33, t2b = 10.33, t2e = 10.8;
    //Time t1b = 10.33, t1e = 10.8, t2b = 11.85, t2e = 12.30;
    //Time t1b = 11.85, t1e = 12.30, t2b = 13.24, t2e = 13.80;

    //ImuTestResult res = testImuFromFile("dump-mpu9250-1.csv", 11.85, 12.30, 13.24, 13.80, 1.0, 0.1);
    testImuFromFile("dump-mpu9250-x-10.csv", 27.10, 27.80, 30.60, 32.00, 1.0, 0.1);
    //ImuTestResult res = testImuFromFile("dump-mpu9250-x10r.csv", 39.30, 39.90, 42.80, 44.40, 0.05, 0.05);
    //ImuTestResult res = testImuFromFile("dump-mpu9250-r180.csv", 82.5, 82.9, 85.3, 88.0, 1.0, 1.0);
    //Eigen::Quaterniond qq;
    //qq.setFromTwoVectors(a2, v0);

    std::cout << "l1 << l2";
}

TEST(CopterImuTest, fromGravityAndMag)
{
    typedef double Time;
    typedef double Scalar;
    typedef ImuPoint<Time, Scalar> Point;
    Eigen::Vector3d grav, mag;
    Point::Measure m;

    /*Point p(0.0, m);
    grav << 0.0, 0.0, 1.0;
    mag << 1.0, 0.0, 0.0;
    p.fromGravityAndMag(grav, mag);*/

    m.w.setZero();
    grav << 1, -2, 9;
    mag << 5, 7, 1;
    m.a = grav;
    Point p1(0.0, m), p2(1.0, m);
    ImuSequence<Time, Scalar> seq;
    seq.setGravity(grav.norm());
    p1.state.p.setZero();
    p1.state.v.setZero();
    p1.fromGravityAndMag(grav, mag);
    p2.from(std::make_pair(&p1, &seq), &seq);

    EXPECT_EQ(p1.state.q.x(), p2.state.q.x());
    EXPECT_EQ(p1.state.q.y(), p2.state.q.y());
    EXPECT_EQ(p1.state.q.z(), p2.state.q.z());
    EXPECT_EQ(p1.state.q.w(), p2.state.q.w());

    EXPECT_NEAR(p2.state.v.norm(), 0, 1E-14);
}
