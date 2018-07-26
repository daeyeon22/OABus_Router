#ifndef __ILP_H__
#define __ILP_H__
#include <iostream>
#include <vector>
#include <climits>
#include <google/dense_hash_map>

using google::dense_hash_map;
using namespace std;

namespace OABusRouter
{
    struct Candidate
    {
        int deg;
        dense_hash_map<int,int> lmapping;

        Candidate(int deg, int* vars, int* vals) :
            deg(deg)
        {
            lmapping.set_empty_key(0);
            for(int i=0; i < deg; i++)
                lmapping[vars[i]] = vals[i];
        }

        int operator [] (int key)
        {
            return lmapping[key];
        }

    };

    struct Clip
    {   
        int id;
        int busid;
        int treeid;
        int numSegs;
        int numCandi;

        vector<int> segs; 
        vector<Candidate> candi;

        Clip():
            id(INT_MAX),
            busid(INT_MAX),
            treeid(INT_MAX),
            numSegs(INT_MAX),
            numCandi(INT_MAX)
        {}

        Candidate& operator [] (int index)
        {   
            return candi[index];
        }

    };
};


#endif



