#ifndef SEQUENCESTD_H
#define SEQUENCESTD_H
#include <map>
#include <deque>
#include <assert.h>

template<class Time, class Data>
class SequenceMap : public std::map<Time, Data>
{
public:
    class Iterator : public std::map<Time, Data>::iterator
    {
    public:

    };

    inline typename std::map<Time, Data>::iterator findEqAfter(Time time) { return lower_bound(time); }
    inline typename std::map<Time, Data>::iterator findEqBefore(Time time)
        {
        typename std::map<Time, Data>::iterator i = lower_bound(time);
        //if(i == end()) return this->
        }
        inline void add(Time time, const Data &data) { insert(std::make_pair(time, data)); }
        //inline const
};

template<class Time>
class Stamped
{
    const Time time_;
public:
    inline Time time() const { return time_; }
    Stamped(Time time) : time_(time) { }
};

/*template<class Time, class Data>
class Stamped : public Data
{
    Time time_;
public:
    inline Time time() const { return time_;}
    template<class T>
    Stamped(Time time, const T &data) : Data(data), time_(time) {}
};*/

template<class Data>
class SequenceDeque : public std::deque<Data>
{
public:
    typedef std::deque<Data> Deque;
    typedef typename Deque::iterator iterator;
    template<class Time>
    iterator findBefore(Time time)
    {
        iterator r = Deque::end();
        for(auto x = Deque::begin(); x != Deque::end(); x++)
        {
            if(x->time() < time) r = x;
            else break;
        }
        return r;
    }
    /*Iterator findAfter(Time time)
    {
        return Deque::begin();
    }*/
    template<typename... _Args>
    inline void add(_Args&&... __args)
    {
        Deque::emplace_back(__args...);
        assert(Deque::size() <= 1 || Deque::back().time() > Deque::at(Deque::size() - 2).time());
    }
    static inline iterator sub1(const iterator &i) { return i - 1; }
    static inline void inc(iterator &i) { i++; }
};


#endif // SEQUENCESTD_H
