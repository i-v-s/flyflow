#ifndef SEQUENCELOOP_H
#define SEQUENCELOOP_H
#include <assert.h>

/***************************************

    Класс для хранения последовательных однотипных данных
  в циклическом буфере с возможностью поиска по времени

****************************************/

template<typename Data, int SIZE>
class SequenceLoop
{
private:
    struct Item { char buf[sizeof(Data)]; };
    Item data[SIZE];
    Item * volatile src, * volatile dst;
    template<typename Time> Data * findBeforeIn(Time time, Data *begin, Data *end)
    {
        auto left = begin, right = end - 1;
        while( left < right )
        {
            Data *mid = left + (right - left) / 2;
            if(mid->time() < time) left = mid + 1;
            else                  right = mid - 1;
        }
        while( left->time() >= time )
        {
            left--;
            if(left < begin) return nullptr;
        }
        return left;
    }
public:
    SequenceLoop() : src(data), dst(data) {}
    ~SequenceLoop()
    {
        for(Item *s = src, *d = dst; s != d;)
        {
            ((Data *)s++)->~Data();
            if(s >= data + SIZE) s = data;
        }
    }
    inline bool empty() { return src == dst; }
    template <class... Args> Data * push(Args&&... args)
    {
        Item *d = dst;
        int empty = (src - d - 1) & (SIZE - 1);
        if(!empty) pop();
        Data * res = new (d++) Data(args...);
        if(d >= data + SIZE) d = data;
        dst = d;
        return res;
    }
    void pop()
    {
        Item *s = src;
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
    template<typename Time> Data * findBefore(Time t)
    {
        Data *d = (Data *)dst, *s = (Data *)src;
        if(s == d || s->time() > t)
            return nullptr;
        if(s < d)
            return findBeforeIn(t, s, d);
        if(d > (Data *)data && ((Data *)data)->time() < t)
            return findBeforeIn(t, (Data *) data, d);
        return findBeforeIn(t, s, (Data *)(data + SIZE));
    }
    inline Data * last()
    {
        Item *s = src;
        if(s == dst) return nullptr;
        return (Data *) s;
    }
    inline Data * first()
    {
        Item *d = dst;
        if(d == src) return nullptr;
        if(--d < data) d += SIZE;
        return (Data *) d;
    }};

#endif // SEQUENCELOOP_H
