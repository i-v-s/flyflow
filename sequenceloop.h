#ifndef SEQUENCELOOP_H
#define SEQUENCELOOP_H
#include <assert.h>
#include <type_traits>

/***************************************

    Класс для хранения последовательных однотипных данных
  в циклическом буфере с возможностью поиска по времени

****************************************/

template<typename Data, int SIZE = 64>
class SequenceLoop
{
private:
    typedef typename std::aligned_storage<sizeof(Data), alignof(Data)>::type Place;
    Place data[SIZE];
    Place * volatile src, * volatile dst;
    template<typename Time> Data * findBeforeIn(Time time, Data *b, Data *e)
    {
        auto left = b, right = e - 1;
        while( left < right )
        {
            Data *mid = left + (right - left) / 2;
            if(mid->time() < time) left = mid + 1;
            else                  right = mid - 1;
        }
        while( left->time() >= time )
        {
            left--;
            if(left < b) return end();
        }
        return left;
    }
public:
    typedef Data value_type;
    typedef Data *iterator;
    SequenceLoop() : src(data), dst(data) {}
    ~SequenceLoop()
    {
        for(Place *s = src, *d = dst; s != d;)
        {
            ((Data *)s++)->~Data();
            if(s >= data + SIZE) s = data;
        }
    }
    inline bool empty() const { return src == dst; }
    template <class... Args> Data * add(Args&&... args)
    {
        Place *d = dst;
        int empty = (src - d - 1) & (SIZE - 1);
        if(!empty) drop();
        Data * res = new (d++) Data(args...);
        if(d >= data + SIZE) d = data;
        dst = d;
        return res;
    }
    void drop()
    {
        Place *s = src;
        if(s == dst)
        {
            assert(!"SequenceLoop::pop(): empty buffer");
            return;
        }
        ((Data *)s)->~Data();
        s++;
        if(s >= data + SIZE) s = data;
        src = s;
    }
    template<typename Time> iterator findBefore(Time t)
    {
        Data *d = (Data *)dst, *s = (Data *)src;
        if(s == d || s->time() > t)
            return end();
        if(s < d)
            return findBeforeIn(t, s, d);
        if(d > (Data *)data && ((Data *)data)->time() < t)
            return findBeforeIn(t, (Data *) data, d);
        return findBeforeIn(t, s, (Data *)(data + SIZE));
    }
    inline Data * last()
    {
        Place *s = src;
        if(s == dst) return nullptr;
        return (Data *) s;
    }
    inline Data * first()
    {
        Place *d = dst;
        if(d == src) return nullptr;
        if(--d < data) d += SIZE;
        return (Data *) d;
    }
    inline iterator begin() {return (iterator) src;}
    inline iterator end() {return (iterator) dst;}
    inline void inc(iterator &i) const
    {
        if(++i >= (Data *)data + SIZE) i = (Data *)data;
    }
    inline iterator sub1(iterator i) const
    {
        return (i == (Data *)data) ? (Data *)data + SIZE - 1 : (i - 1);
    }
    inline Data &back()             { assert(!empty()); return *sub1(dst); }
    inline const Data &back() const { assert(!empty()); return *sub1((iterator)dst); }
};

#endif // SEQUENCELOOP_H
