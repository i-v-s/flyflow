#ifndef TIMEFUSION_H
#define TIMEFUSION_H
#include <utility>
#include <cstddef>
#include <assert.h>

template<class Time = double, typename... Other> class TimeFusion;
template<class Time> class TimeFusion<Time>
{
public:
    template<class State> constexpr bool get(State *, Time *) const { return false; }
protected:
    inline void setPointBefore(Time) {}
    template<class Measure> void addMeasure(Time, const Measure &) { assert(!"Unknown Measure type"); }
    template<class Measure, class It>
    void addMeasure(Time, const Measure &, const It &)
    {
        assert(!"Unknown Measure type");
    }
    constexpr bool initCalculate(Time, Time) { return false; }
    template<class Pair1> void addMeasureIt(const Pair1 &i)
    { // Не нашли предыдущего состояния
        i.first->from(i.second);
    }

    template<class Pair1, class Pair2>
    void addMeasureIt(const Pair1 &i, const Pair2 &bestIt)
    {
        i.first->from(bestIt, i.second);
    }

    template<class Pair1, class Pair2> void calc(Pair1 i, Pair2 o)
    {
        o.first->from(i, o.second);
    }
    template<class It1> inline void calc(const It1 &, std::nullptr_t) {}
    template<class It2> inline void calc(std::nullptr_t, const It2 &) { assert(!"Unable to calc"); }
    inline void calc(std::nullptr_t, std::nullptr_t) {}
};

template<class Time, typename Sequence, typename... Other>
class TimeFusion<Time, Sequence, Other...> : private TimeFusion<Time, Other...>
{
private:
    typedef TimeFusion<Time, Other...> Parent;
    typedef typename Sequence::value_type Point;
    typedef typename Sequence::iterator Iterator;
    Sequence points;
    Iterator point;
protected:
// addMeasure[It] - добавить измерение, инициализировать point
    template<class Pair1, class Pair2>
    void addMeasureIt(const Pair1 &i, const Pair2 &bestIt)
    {
        point = points.findBefore(i.first->time());
        if(point == points.end())
        {
            Parent::addMeasureIt(i, bestIt);
            point = points.begin();
        }
        else
        {
            if(bestIt.first->time() > point->time())
                Parent::addMeasureIt(i, bestIt);
            else
                Parent::addMeasureIt(i, std::make_pair(point, &points));
            points.inc(point);
        }
    }
    template<class Pair1>
    void addMeasureIt(const Pair1 &i)
    {
        point = points.findBefore(i.first->time());
        if(point == points.end())
            Parent::addMeasureIt(i);
        else
        {
            Parent::addMeasureIt(i, std::make_pair(point, &points));
            points.inc(point);
        }
    }
    template<class AnyIterator>
    void addMeasure(Time time, const typename Point::Measure &measure, const AnyIterator &bestIt) // Добавляем измерение, тип которого совпадает с верхним
    {
        points.add(time, measure);
        point = points.end();
        Parent::addMeasureIt(std::make_pair(points.sub1(point), &points), bestIt);
    }
    void addMeasure(Time time, const typename Point::Measure &measure) // Добавляем измерение, тип которого совпадает с верхним
    {
        if(points.empty())
        {
            points.add(time, measure);
            point = points.end();
            Parent::addMeasureIt(std::make_pair(points.sub1(point), &points));
        }
        else
        {
            points.add(time, measure);
            point = points.end();
            Iterator last = points.sub1(point);
            Parent::addMeasureIt(std::make_pair(last, &points), std::make_pair(points.sub1(last), &points));
        }
    }
    template<class Measure>
    void addMeasure(Time time, const Measure &measure) // Добавляем измерение, тип которого не совпадает с верхним
    {
        point = points.findBefore(time);
        if(point == points.end())
        {
            Parent::addMeasure(time, measure);
            point = points.begin();
        }
        else
        {
            Parent::addMeasure(time, measure, std::make_pair(point, &points));
            points.inc(point);
        }
    }
    template<class Measure, class Pair>
    void addMeasure(Time time, const Measure &measure, const Pair &bestIt) // Добавляем измерение, тип которого не совпадает с верхним
    {
        point = points.findBefore(time);
        if(point == points.end())
        {
            Parent::addMeasure(time, measure, bestIt);
            point = points.begin();
        }
        else
        {
            if(bestIt.first->time() > point->time())
                Parent::addMeasure(time, measure, bestIt);
            else
                Parent::addMeasure(time, measure, std::make_pair(point, &points));
            points.inc(point);
        }
    }

    ////////////// i позднее, чем point-1 ? ///////////////////
    inline bool laterThanKnown(std::nullptr_t) { return point == points.begin(); }
    template<class T> inline bool laterThanKnown(T i)
    {
        if(point == points.begin()) return true;
        return i.first->time() > points.sub1(point)->time();
    }
    ////////////// можно ли считать point?
    inline bool pointOk(std::nullptr_t) { return point != points.end(); }
    template<class T> inline bool pointOk(T o)
    {
        if(point == points.end()) return false;
        return point->time() < o.first->time();
    }
    // Расчитать из состояния i состояние o
    template<class Pair1, class Pair2>
    void calc(const Pair1 &i, const Pair2 &o)
    {
        if(points.empty()) { Parent::calc(i, o); return; }

        if(pointOk(o))
        {
            if(laterThanKnown(i))
                Parent::calc(i, std::make_pair(point, &points));
            else
                Parent::calc(std::make_pair(points.sub1(point), &points), std::make_pair(point, &points));
        }
        else
        {
            if(laterThanKnown(i))
                Parent::calc(i, o);
            else
                Parent::calc(std::make_pair(points.sub1(point), &points), o);
            return;
        }
        Iterator prev = point;
        for(points.inc(point); pointOk(o); points.inc(point))
        {
            Parent::calc(std::make_pair(prev, &points), std::make_pair(point, &points));
            prev = point;
        }

        Parent::calc(std::make_pair(prev, &points), o);
    }
public:
    template<class Measure>
    void onMeasure(Time time, const Measure &measure)
    {
        addMeasure(time, measure);
        calc(nullptr, nullptr);
    }

    // Возвращает последнее состояние
    template<class State>
    inline bool get(State *state, Time *time) const
    {
        return Parent::get(state, time);
    }
    bool get(typename Point::State *state, Time *time) const
    {
        if(points.empty()) return false;
        auto p = points.back();
        *time = p.time();
        *state = p.state;
        return true;
    }
};



#endif // TIMEFUSION_H
