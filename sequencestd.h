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

template<class Time, class Data>
struct Stamped : public Data
{
    Time time_;
public:
    inline Time time() const { return time_;}
    template<class T>
    Stamped(Time time, const T &data) : Data(data), time_(time) {}
};

template<class Time, class Data>
class SequenceDeque : public std::deque<Stamped<Time, Data>>
{
public:
    typedef std::deque<Stamped<Time, Data>> Deque;
    typedef typename Deque::iterator iterator;
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
    template<class T>
    inline void add(Time time, const T &data)
    {
        assert(Deque::empty() || Deque::back().time() < time);
        Deque::emplace_back(time, data);
    }
    static inline iterator sub1(const iterator &i) { return i - 1; }
    static inline void inc(iterator &i) { i++; }
};


#endif // SEQUENCESTD_H
