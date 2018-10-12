#ifndef __TREE_H__
#define __TREE_H__

#include "typedef.h"

namespace OABusRouter
{

    struct RtreeNode
    {
        int id;
        int offset;
        int l;
        int width;
        int x1, x2;
        int y1, y2;
        bool vertical;
        
        RtreeNode *upper, *lower;

        seg elem;

        vector<int> neighbor;
        dense_hash_map<int,pair<int,int>> intersection;

        RtreeNode() :
            id(INT_MAX),
            offset(INT_MAX),
            l(INT_MAX),
            width(INT_MAX),
            x1(INT_MAX),
            x2(INT_MAX),
            y1(INT_MAX),
            y2(INT_MAX),
            vertical(false)
        {
            intersection.set_empty_key(INT_MAX);
            upper = nullptr;
            lower = nullptr;
        }


        RtreeNode(const RtreeNode& i) :
            id(i.id),
            offset(i.offset),
            l(i.l),
            width(i.width),
            upper(i.upper),
            lower(i.lower),
            x1(i.x1),
            x2(i.x2),
            y1(i.y1),
            y2(i.y2),
            vertical(i.vertical),
            elem(i.elem),
            neighbor(i.neighbor),
            intersection(i.intersection)
        {}
    };

    struct Rtree_t
    {
        set<edge> edges;
        vector<SegRtree> trees;
        vector<RtreeNode> nodes;


        bool lower(RtreeNode* n1, RtreeNode* n2);
        bool upper(RtreeNode* n1, RtreeNode* n2);

        RtreeNode* get_node(int trackid)
        {
            return &nodes[trackid]; 
        }
        
        SegRtree* operator [](int l)
        { 
            return &trees[l]; 
        }

    };

    struct Rtree_o
    {
        int db[4];
        vector<BoxRtree> pins;
        vector<BoxRtree> obstacles;
        vector<BoxRtree> wires;

        Rtree_o()
        {}

       

        bool insert(int type, int bitid, int x[], int y[], int l);
        bool remove(int type, int bitid, int x[], int y[], int l);
        int num_spacing_violation(int bitid, int x[], int y[], int l, int width, int spacing, bool vertical);
        
    };


};


#endif
