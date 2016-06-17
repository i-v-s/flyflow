#include "timefusion.h"
#include "sequenceloop.h"
#include "sequencestd.h"
#include <gtest/gtest.h>

struct TestData
{
    const TestData *prev;
    double dt;
    int n, m, g;
};

template<int N>
class TestPoint
{
public:
    struct Measure
    {
        int x;
        Measure(int x) : x(x) { }
        //Measure() { }
    } measure;
    struct State
    {
        TestData data;
        //constexpr int n() {return N;}
        State() : data({nullptr, 0.0, -1, -1, -1}) { }
    } state;
    template<class AnyState>
    void from(const AnyState &prev, double dt) // Обновиться, опираясь на предыдущее состояние, бывшее dt секунд назад
    {
        state.data.m = measure.x;
        state.data.n = N;
        state.data.prev = &prev.data;
        state.data.dt = dt;
        state.data.g = prev.data.g + 1;
    }
    void from() // Обновиться, опираясь только на измерение
    {
        state.data.prev = nullptr;
        state.data.n = N;
        state.data.m = measure.x;
        state.data.dt = 0.0;
        state.data.g = 0;
    }
    TestPoint(const Measure &measure) : measure(measure) {}
};

TEST(FusionTest, single)
{
    TestPoint<1>::State s;
    double t = -1;
    TimeFusion<double, SequenceDeque<double, TestPoint<1>>> tf;
    EXPECT_FALSE(tf.get(&s, &t));
    tf.onMeasure(0.0, TestPoint<1>::Measure('a'));
    EXPECT_TRUE(tf.get(&s, &t));
    //EXPECT_DOUBLE_EQ(s.x, 0.0);
    EXPECT_DOUBLE_EQ(t, 0.0);

    tf.onMeasure(1.0, TestPoint<1>::Measure('b'));
    EXPECT_TRUE(tf.get(&s, &t));
    //EXPECT_DOUBLE_EQ(s.x, 0.75);
    EXPECT_DOUBLE_EQ(t, 1.0);
}

template<char A>
class TextPoint
{
public:
    struct Measure
    {
        int x;
        Measure(int x) : x(x) { }
        //Measure() { }
    } measure;
    struct State
    {
        std::string data;
        State() : data("?") { }
    } state;
    TextPoint(const Measure &measure) : measure(measure) {}
    void from()
    {
        char b[20];
        sprintf(b, "%c%d", A, measure.x);
        state.data = b;
    }
    template<class AnyState>
    void from(const AnyState &prev, int dt) // Обновиться, опираясь на предыдущее состояние, бывшее dt секунд назад
    {
        char b[20];
        sprintf(b, "(%d)%c%d", dt, A, measure.x);
        state.data = prev.data + b;
    }
};

template<typename Time, class Point>
class Sequence : public SequenceLoop<Stamped<Time, Point>> {};

TEST(FusionTest, three)
{
    TextPoint<'A'>::State a;
    TextPoint<'B'>::State b;
    TextPoint<'C'>::State c;
    int t;
    typedef int Time;
    typedef Sequence<Time, TextPoint<'A'>> SeqA;
    typedef Sequence<Time, TextPoint<'B'>> SeqB;
    typedef Sequence<Time, TextPoint<'C'>> SeqC;
    TimeFusion<Time, SeqA, SeqB, SeqC> tf;
                                                 // 0  1  2  3  4  5
    tf.onMeasure(1, TextPoint<'A'>::Measure(1)); //   >A1
    EXPECT_TRUE(tf.get(&a, &t));
    EXPECT_EQ(a.data, "A1");
    tf.onMeasure(4, TextPoint<'B'>::Measure(2)); //    A1      >B2
    EXPECT_TRUE(tf.get(&b, &t));
    EXPECT_EQ(b.data, "A1(3)B2");
    tf.onMeasure(0, TextPoint<'C'>::Measure(3)); //>C3 A1       B2
    EXPECT_TRUE(tf.get(&c, &t));
    EXPECT_EQ(c.data, "C3");
    EXPECT_TRUE(tf.get(&b, &t));
    EXPECT_EQ(b.data, "C3(1)A1(3)B2");
    tf.onMeasure(2, TextPoint<'A'>::Measure(4)); // C3 A1>A4    B2
    EXPECT_TRUE(tf.get(&a, &t));
    EXPECT_EQ(a.data, "C3(1)A1(1)A4");
    EXPECT_TRUE(tf.get(&b, &t));
    EXPECT_EQ(b.data, "C3(1)A1(1)A4(2)B2");
    tf.onMeasure(3, TextPoint<'C'>::Measure(5)); // C3 A1 A4>C5 B2
    EXPECT_TRUE(tf.get(&c, &t));
    EXPECT_EQ(c.data, "C3(1)A1(1)A4(1)C5");
    EXPECT_TRUE(tf.get(&b, &t));
    EXPECT_EQ(b.data, "C3(1)A1(1)A4(1)C5(1)B2");
    tf.onMeasure(5, TextPoint<'C'>::Measure(6)); // C3 A1 A4 C5 B2>C6
    EXPECT_TRUE(tf.get(&c, &t));
    EXPECT_EQ(c.data, "C3(1)A1(1)A4(1)C5(1)B2(1)C6");
}

/*TEST(FusionTest, bruteforce)
{
    //TestPoint1::State s;
    //double t = -1;
    for(int seed = 1; seed < 0xFFFF; seed++)
    {
        TimeFusion<double, TestPoint<0>, TestPoint<1>, TestPoint<2>, TestPoint<3>> tf;
        struct Data { int tp; double time;} data[10]; // тип измерения, промежуток
        int size = 10;
        double time = 0.0;
        for(int x = 0; x < size; x++) // Генерируем тестовые измерения
        {
            int cs = (seed << (x & 15)) | (seed >> (16 - (x & 15)));
            Data &d = data[x];
            d.tp = (x + cs) & 3;
            d.time = ((x + cs) & 15) * 0.5 + 0.1;
        }

        int ctr[4] = {0};
        for(int est = 4, x = 0; est; x += 13) // Добавляем их в произвольном порядке
        {
            int cs = (seed << (x & 15)) | (seed >> (16 - (x & 15)));
            int tp = (x + cs) & 3;
            while(ctr[tp] == size)
                tp = (tp + 1) & 3;
            while(ctr[tp] < size && data[ctr[tp]].tp != tp) ctr[tp]++;
            if(ctr[tp] == size)
            {
                est--;
                continue;
            }
            double dt = data[ctr[tp]].time;
            switch(tp)
            {
                case 0: tf.onMeasure(time, TestPoint<0>::Measure(ctr[0]++)); break;
                case 1: tf.onMeasure(time, TestPoint<1>::Measure(ctr[1]++)); break;
                case 2: tf.onMeasure(time, TestPoint<2>::Measure(ctr[2]++)); break;
                case 3: tf.onMeasure(time, TestPoint<3>::Measure(ctr[3]++)); break;
            }
            time += dt;
            if(ctr[tp] == size) est--;
        }
        //for(int tp = 0; tp < 4; tp++)
        {
            TestPoint<0>::State s;
            double t;
            EXPECT_TRUE(tf.get(&s, &t));
            const TestData *td = &s.data;
            int x = size - 1;
            for(; x >= 0; x--) if(data[x].tp == 0) break;
            for(; x >= 0 && td; x--)
            {
                EXPECT_TRUE(!x || td->prev);
                EXPECT_EQ(data[x].tp, td->n);
                td = td->prev;
            }
            EXPECT_FALSE(td);
        }

    }
}*/
