#ifndef __ILP_H__
#define __ILP_H__

namespace OABusRouter
{
    class Solver
    {
        dense_hash_map<int,int> bus2shape;


        int get_routing_shape(int mp1, int mp2);


    };

    struct Clip
    {
        int busid;
        string shape;


    };



};


#endif


