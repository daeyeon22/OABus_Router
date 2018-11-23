#include "heap.h"

template <class A>
void OABusRouter::Heap<A>::push(vector<A>& next)
{
    for(int i=0; i < next.size(); i++)
    {
        PQ.push(next[i]);
    }
}

template <class A>
void OABusRouter::Heap<A>::push(A& next)
{
    PQ.push(next);
}

template <class A>
void OABusRouter::Heap<A>::pop()
{
    PQ.pop();
}

template <class A>
A OABusRouter::Heap<A>::top()
{
    return PQ.top();
}

template <class A>
bool OABusRouter::Heap<A>::empty()
{
    return PQ.size() == 0 ? true : false;
}

template <class A>
int OABusRouter::Heap<A>::size()
{
    return PQ.size();
}


