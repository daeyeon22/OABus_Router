#ifndef __TREE_H__
#define __TREE_H__

#include "typedef.h"

namespace OABusRouter
{

    struct Rtree_o
    {
        int db[4];
        vector<BoxRtree> pins;
        vector<BoxRtree> obstacles;
        vector<BoxRtree> wires;
        vector<SegRtree> empties;

        Rtree_o()
        {}

        bool insert(int type, int bitid, int x1, int y1, int x2, int y2, int l);
        bool remove(int type, int bitid, int x1, int y1, int x2, int y2, int l);
        bool insert(int type, int bitid, int x1, int y1, int x2, int y2, int l, int width);
        bool remove(int type, int bitid, int x1, int y1, int x2, int y2, int l, int width);
        bool insert(int type, int bitid, int x[], int y[], int l);
        bool remove(int type, int bitid, int x[], int y[], int l);
        bool spacing_violation_clean(int bitid, int x1, int y1, int x2, int y2, int l, int width, int spacing, bool vertical);
        int num_spacing_violation(int bitid, int x[], int y[], int l, int width, int spacing, bool vertical);
        int num_spacing_violation(int bitid, int x1, int y1, int x2, int y2, int l, int width, int spacing, bool vertical);
        bool obstacle_clean(int x, int y, int l, int width, int spacing);
    };
    
    struct Constraint
    {
        int min_width;
        int max_width;
        vector<int> widths;
        dense_hash_map<int, bi::interval_set<int>> constraint;
    
        Constraint() :
            min_width(INT_MAX),
            max_width(INT_MIN)
        {
            constraint.set_empty_key(INT_MAX);
        }

        Constraint(const Constraint& c) :
            min_width(c.min_width),
            max_width(c.max_width),
            widths(c.widths),
            constraint(c.constraint)
        {}
    };

    
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
        RtreeNode *prev, *next;

        vector<int> widths;
        vector<int> neighbor;
        dense_hash_map<int,pair<int,int>> intersection;
        //dense_hash_map<int,bi::interval_set<int>> constraint;
        //bi::interval_map<int,int> constraint; //width_const; //raint

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
            prev = nullptr;
            next = nullptr;
        }

        RtreeNode(const RtreeNode& i) :
            id(i.id),
            offset(i.offset),
            l(i.l),
            width(i.width),
            x1(i.x1),
            x2(i.x2),
            y1(i.y1),
            y2(i.y2),
            vertical(i.vertical),
            upper(i.upper),
            lower(i.lower),
            prev(i.prev),
            next(i.next),
            neighbor(i.neighbor),
            intersection(i.intersection)
        {}

    };


    struct Rtree_t
    {
        set<edge> edges;
        vector<SegRtree> trees;
        vector<RtreeNode> nodes;
        vector<Constraint> constraints;


        bool is_valid(int n1, int n2);
        bool is_valid(pair<int,int> e);
        bool corner(int current, int &target, int l, bool lCorner);
        //bool next(int current, int &next, int neighbor, int width, int spacing, bool upper);
        bool search_node_debug(int current, int &next, int neighbor, int width, int spacing, bool upper);
        bool search_node(int current, int &next, int neighbor, int width, int spacing, bool upper);
        bool search_node_SPV_clean(int bitid, int current, int &next, int neighbor, int width, int spacing, bool upper, Rtree_o& rtree_o);
        bool search_node(int current, int &next, int width, int spacing, bool upper);
        bool search_node2(int current, int &next, int neighbor, int width, int spacing, bool upper);
        bool search_node(RtreeNode* n1, RtreeNode* n2, RtreeNode* n_iter, int width, int spacing, bool upper);
        bool search_node2(RtreeNode* n1, RtreeNode* n2, RtreeNode* n_iter, int width, int spacing, bool upper);
        bool stacked_node(int n1, int &n2, int x, int y, int l); 

        bool next_node(int bitid, int n_iter, int n1, int &n2, int w1, int w2, int sp, int offset, int x1, int y1, bool upper, Rtree_o& rtree_o);
    

        bool get_segments(int current, int width, int spacing, bool upper, vector<int> &ns);


        bool get_extension(int n1, int& n2, bool lCorner);
        bool prev(int current, int &previous);
        bool next(int current, int &next);
        bool lower(RtreeNode* n1, RtreeNode* n2, int prev, int width, int spacing);
        bool upper(RtreeNode* n1, RtreeNode* n2, int prev, int width, int spacing);

        bool maximum_width_constraint(int n, int p1, int p2, int width);
        
        
        RtreeNode* get_node(int trackid)
        {
            return &nodes[trackid]; 
        }
        
        SegRtree* operator [](int l)
        { 
            return &trees[l]; 
        }

    };

};


#endif
