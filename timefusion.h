#ifndef TIMEFUSION_H
#define TIMEFUSION_H
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
    template<class It1> void addMeasureIt(const It1 &i)
    { // Не нашли предыдущего состояния
        i->from();
    }

    template<class Iterator1, class Iterator2>
    void addMeasureIt(const Iterator1 &i, const Iterator2 &bestIt)
    {
        i->from(bestIt->state, i->time() - bestIt->time());
    }

    template<class It1, class It2> void calc(It1 i, It2 o)
    {
        o->from(i->state, o->time() - i->time());
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
    template<class Iterator1, class Iterator2>
    void addMeasureIt(const Iterator1 &i, const Iterator2 &bestIt)
    {
        point = points.findBefore(i->time());
        if(point == points.end())
        {
            Parent::addMeasureIt(i, bestIt);
            point = points.begin();
        }
        else
        {
            if(bestIt->time() > point->time())
                Parent::addMeasureIt(i, bestIt);
            else
                Parent::addMeasureIt(i, point);
            points.inc(point);
        }
    }
    template<class Iterator1>
    void addMeasureIt(const Iterator1 &i)
    {
        point = points.findBefore(i->time());
        if(point == points.end())
            Parent::addMeasureIt(i);
        else
        {
            Parent::addMeasureIt(i, point);
            points.inc(point);
        }
    }
    template<class AnyIterator>
    void addMeasure(Time time, const typename Point::Measure &measure, const AnyIterator &bestIt) // Добавляем измерение, тип которого совпадает с верхним
    {
        points.add(time, measure);
        point = points.end();
        Parent::addMeasureIt(points.sub1(point), bestIt);
    }
    void addMeasure(Time time, const typename Point::Measure &measure) // Добавляем измерение, тип которого совпадает с верхним
    {
        if(points.empty())
        {
            points.add(time, measure);
            point = points.end();
            Parent::addMeasureIt(points.sub1(point));
        }
        else
        {
            points.add(time, measure);
            point = points.end();
            Iterator last = points.sub1(point);
            Parent::addMeasureIt(last, points.sub1(last));
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
            Parent::addMeasure(time, measure, point);
            points.inc(point);
        }
    }
    template<class Measure, class AnyIterator>
    void addMeasure(Time time, const Measure &measure, const AnyIterator &bestIt) // Добавляем измерение, тип которого не совпадает с верхним
    {
        point = points.findBefore(time);
        if(point == points.end())
        {
            Parent::addMeasure(time, measure, bestIt);
            point = points.begin();
        }
        else
        {
            if(bestIt->time() > point->time())
                Parent::addMeasure(time, measure, bestIt);
            else
                Parent::addMeasure(time, measure, point);
            points.inc(point);
        }
    }

    ////////////// i позднее, чем point-1 ? ///////////////////
    inline bool laterThanKnown(std::nullptr_t) { return point == points.begin(); }
    template<class T> inline bool laterThanKnown(T i)
    {
        if(point == points.begin()) return true;
        return i->time() > points.sub1(point)->time();
    }
    ////////////// можно ли считать point?
    inline bool pointOk(std::nullptr_t) { return point != points.end(); }
    template<class T> inline bool pointOk(T o)
    {
        if(point == points.end()) return false;
        return point->time() < o->time();
    }
    // Расчитать из состояния i состояние o
    template<class It1, class It2>
    void calc(It1 i, It2 o)
    {
        if(points.empty()) { Parent::calc(i, o); return; }

        if(pointOk(o))
        {
            if(laterThanKnown(i))
                Parent::calc(i, point);
            else
                Parent::calc(points.sub1(point), point);
        }
        else
        {
            if(laterThanKnown(i))
                Parent::calc(i, o);
            else
                Parent::calc(points.sub1(point), o);
            return;
        }
        Iterator prev = point;
        for(points.inc(point); pointOk(o); points.inc(point))
        {
            Parent::calc(prev, point);
            prev = point;
        }

        Parent::calc(prev, o);
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
