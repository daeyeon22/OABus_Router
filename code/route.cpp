#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <set>
#include "func.h"
#include "circuit.h"
#include "route.h"



bool OABusRouter::Router::get_middle_seg(int s1, int s2, int &tar)
{
    bool exist = false;
    Segment* seg1 = &segs[s1];
    for(auto& mid : seg1->neighbor)
    {
        Segment* middle = &segs[mid];
        for(auto& last : middle->neighbor)
        {
            if(last == s2)
            {
                tar = middle->id;
                exist = true;
                break;
            }
        }

        if(exist)
            break;
    }

    return exist;
}



bool OABusRouter::Router::should_stop()
{
    return ckt->should_stop();
}

void OABusRouter::Router::wire_reordering(int busid, vector<Segment> &tp)
{

    if(tp.size() < 2)
        return;

    int i, j,l1, l2;
    box bb1, bb2;

    for(i=0; i < tp.size()-2; i++)
    {
        Segment *seg1, *seg2;
        seg1 = &tp[i];
        seg2 = &tp[i+2];
        l1 = seg1->l;
        l2 = seg2->l;
        bb1 = box(pt(seg1->x1, seg1->y1), pt(seg1->x2, seg1->y2));
        bb2 = box(pt(seg2->x1, seg2->y1), pt(seg2->x2, seg2->y2));

        vector<pair<box, int>> queries1;
        vector<pair<box, int>> queries2;

        rtree_s[l1]->query(bgi::overlaps(bb1), back_inserter(queries1));
        rtree_s[l2]->query(bgi::overlaps(bb2), back_inserter(queries2));

        if(queries1.size() == 0 || queries2.size() == 0)
            continue;

        int s1 = INT_MAX, s2 = INT_MAX;
        int busid1 = INT_MAX, busid2 = INT_MAX;
        bool valid = false;
        for(auto& it1 : queries1)
        {

            valid = false;
            s1 = it1.second;
            busid1 = rtree_s.elem2bus[it1.second];
            if(busid1 == busid)
                continue;

            for(auto& it2 : queries2)
            {
                s2 = it2.second;
                busid2 = rtree_s.elem2bus[it2.second];
                if(busid2 == busid1)
                {
                    valid = true;
                    break;
                }
            }

            if(!valid)
                continue;

            Segment *begin1, *begin2, *mid1, *mid2, *last1, *last2;
            int midid;
            if(!get_middle_seg(s1, s2, midid))
                continue;
            
            begin1 = &tp[i];
            mid1 = &tp[i+1];
            last1 = &tp[i+2];
            begin2 = &segs[s1];
            mid2 = &segs[midid];
            last2 = &segs[s2];

            if(mid1->l != mid2->l)
                continue;


            vector<int> offset1;
            vector<int> offset2;
            dense_hash_map<int,int> offset2bit;
            dense_hash_map<int,int> bit2index;
            dense_hash_map<int,int> offset2track;
            offset2bit.set_empty_key(INT_MAX);
            bit2index.set_empty_key(INT_MAX);
            offset2track.set_empty_key(INT_MAX);

            for(auto& wireid : begin1->wires)
            {
                int offset = ckt->tracks[wires[wireid].trackid].offset;
                int bitid = wires[wireid].bitid;
                offset2bit[offset] = bitid;
                offset1.push_back(offset);
            }

            for(auto& wireid : begin2->wires)
            {
                int offset = ckt->tracks[wires[wireid].trackid].offset;
                int bitid = wires[wireid].bitid;
                offset2bit[offset] = bitid;
                offset1.push_back(offset);
            }

            sort(offset1.begin(), offset1.end());
            for(j=0; j < offset1.size(); j++)
                bit2index[offset2bit[offset1[j]]] = j;
    

            for(auto& wireid : mid1->wires)
            {
                int offset = ckt->tracks[wires[wireid].trackid].offset;
                int trackid = wires[wireid].trackid;
                offset2track[offset] = trackid;
                offset2.push_back(offset);
            }

            for(auto& wireid : mid2->wires)
            {
                int offset = ckt->tracks[wires[wireid].trackid].offset;
                int trackid = wires[wireid].trackid;
                offset2track[offset] = trackid;
                offset2.push_back(offset);
            }
            sort(offset2.begin(), offset2.end());
           
            for(auto& wireid : mid1->wires)
            {
                Wire* curw = &wires[wireid];
                int bitid = curw->bitid;
                int offset = offset2[bit2index[bitid]];
                int trackid = offset2track[offset];
                int seq = curw->seq;
                int x, y;
                int xPrev, yPrev;

                if(curw->trackid == trackid)
                    continue;

                rtree_t.insert_element(curw->trackid, curw->x1, curw->y1, curw->x2, curw->y2, curw->l, false);
                curw->trackid = trackid;
                curw->x1 = curw->vertical? offset : curw->x1;
                curw->x2 = curw->vertical? offset : curw->x2;
                curw->y1 = curw->vertical? curw->y1 : offset;
                curw->y2 = curw->vertical? curw->y2 : offset;
                rtree_t.insert_element(curw->trackid, curw->x1, curw->y1, curw->x2, curw->y2, curw->l, true);

                Wire* beginw = &wires[begin1->wires[seq]];
                rtree_t.get_intersection(curw->trackid, beginw->trackid, x, y);
                xPrev = curw->intersection[beginw->id].first;
                yPrev = curw->intersection[beginw->id].second;
                rtree_t.insert_element(beginw->trackid, beginw->x1, beginw->y1, beginw->x2, beginw->y2, beginw->l, false);

                if(xPrev == beginw->x1 && yPrev == beginw->y1)
                {
                    beginw->x1 = x;
                    beginw->y1 = y;
                }
                if(xPrev == beginw->x2 && yPrev == beginw->y2)
                {
                    beginw->x2 = x;
                    beginw->y2 = y;
                }
                
                //beginw->x1 = min(beginw->x1, x);
                //beginw->x2 = max(beginw->x2, x);
                //beginw->y1 = min(beginw->y1, y);
                //beginw->y2 = max(beginw->y2, y);
                beginw->intersection[curw->id] = {x,y};
                curw->intersection[beginw->id] = {x,y};
                rtree_t.insert_element(beginw->trackid, beginw->x1, beginw->y1, beginw->x2, beginw->y2, beginw->l, true);


                Wire* lastw = &wires[last1->wires[seq]];
                xPrev = curw->intersection[lastw->id].first;
                yPrev = curw->intersection[lastw->id].second;
                rtree_t.get_intersection(curw->trackid, lastw->trackid, x, y);
                rtree_t.insert_element(lastw->trackid, lastw->x1, lastw->y1, lastw->x2, lastw->y2, lastw->l, false);

                if(xPrev == lastw->x1 && yPrev == lastw->y1)
                {
                    lastw->x1 = x;
                    lastw->y1 = y;
                }
                if(xPrev == lastw->x2 && yPrev == lastw->y2)
                {
                    lastw->x2 = x;
                    lastw->y2 = y;
                }
                
                //lastw->x1 = min(lastw->x1, x);
                //lastw->x2 = max(lastw->x2, x);
                //lastw->y1 = min(lastw->y1, y);
                //lastw->y2 = max(lastw->y2, y);
                lastw->intersection[curw->id] = {x,y};
                curw->intersection[lastw->id] = {x,y};
                rtree_t.insert_element(lastw->trackid, lastw->x1, lastw->y1, lastw->x2, lastw->y2, lastw->l, true);
    

            }

            for(auto& wireid : mid2->wires)
            {
                Wire* curw = &wires[wireid];
                int bitid = curw->bitid;
                int offset = offset2[bit2index[bitid]];
                int trackid = offset2track[offset];
                int seq = curw->seq;
                int x, y;
                int xPrev, yPrev;
                if(curw->trackid == trackid)
                    continue;

                rtree_t.insert_element(curw->trackid, curw->x1, curw->y1, curw->x2, curw->y2, curw->l, false);
                curw->trackid = trackid;
                curw->x1 = curw->vertical? offset : curw->x1;
                curw->x2 = curw->vertical? offset : curw->x2;
                curw->y1 = curw->vertical? curw->y1 : offset;
                curw->y2 = curw->vertical? curw->y2 : offset;
                rtree_t.insert_element(curw->trackid, curw->x1, curw->y1, curw->x2, curw->y2, curw->l, true);



                Wire* beginw = &wires[begin2->wires[seq]];
                rtree_t.get_intersection(curw->trackid, beginw->trackid, x, y);
                xPrev = curw->intersection[beginw->id].first;
                yPrev = curw->intersection[beginw->id].second;
                rtree_t.insert_element(beginw->trackid, beginw->x1, beginw->y1, beginw->x2, beginw->y2, beginw->l, false);

                if(xPrev == beginw->x1 && yPrev == beginw->y1)
                {
                    beginw->x1 = x;
                    beginw->y1 = y;
                }
                if(xPrev == beginw->x2 && yPrev == beginw->y2)
                {
                    beginw->x2 = x;
                    beginw->y2 = y;
                }
                
                //beginw->x1 = min(beginw->x1, x);
                //beginw->x2 = max(beginw->x2, x);
                //beginw->y1 = min(beginw->y1, y);
                //beginw->y2 = max(beginw->y2, y);
                beginw->intersection[curw->id] = {x,y};
                curw->intersection[beginw->id] = {x,y};
                rtree_t.insert_element(beginw->trackid, beginw->x1, beginw->y1, beginw->x2, beginw->y2, beginw->l, true);


                Wire* lastw = &wires[last2->wires[seq]];
                xPrev = curw->intersection[lastw->id].first;
                yPrev = curw->intersection[lastw->id].second;
                rtree_t.get_intersection(curw->trackid, lastw->trackid, x, y);
                rtree_t.insert_element(lastw->trackid, lastw->x1, lastw->y1, lastw->x2, lastw->y2, lastw->l, false);

                if(xPrev == lastw->x1 && yPrev == lastw->y1)
                {
                    lastw->x1 = x;
                    lastw->y1 = y;
                }
                if(xPrev == lastw->x2 && yPrev == lastw->y2)
                {
                    lastw->x2 = x;
                    lastw->y2 = y;
                }
                
                //lastw->x1 = min(lastw->x1, x);
                //lastw->x2 = max(lastw->x2, x);
                //lastw->y1 = min(lastw->y1, y);
                //lastw->y2 = max(lastw->y2, y);
                lastw->intersection[curw->id] = {x,y};
                curw->intersection[lastw->id] = {x,y};
                rtree_t.insert_element(lastw->trackid, lastw->x1, lastw->y1, lastw->x2, lastw->y2, lastw->l, true);
    
            }
        }
    }
}

void OABusRouter::Router::update_net_tp(int busid, vector<Segment> &tp)
{
    int i, j, s1, s2, x1, x2, y1, y2, l;
    set<pair<int,int>> edges;
    dense_hash_map<int,int> local2global;
    //Topology* top = &topologies[busid];
    local2global.set_empty_key(INT_MAX);
    //top->segs.clear();

    for(i=0; i < tp.size(); i++)
    {
        Segment &seg = tp[i];
        seg.id = segs.size();
        x1 = seg.x1;
        y1 = seg.y1;
        x2 = seg.x2;
        y2 = seg.y2;
        l = seg.l;

        local2global[i] = seg.id;

        for(j=0; j < seg.neighbor.size(); j++)
        {
            s1 = min(i, seg.neighbor[j]);
            s2 = max(i, seg.neighbor[j]);
            if(edges.find({s1, s2}) == edges.end())
                edges.insert({s1, s2});
        }

        segs.push_back(seg);
        // add into the tree
        box bb(pt(x1, y1), pt(x2, y2));
        rtree_s[l]->insert({bb, seg.id});
        rtree_s.elem2bus[seg.id] = busid;
    }


    
    for(auto& it : edges)
    {
        s1 = local2global[it.first];
        s2 = local2global[it.second];

        segs[s1].neighbor.push_back(s2);
        segs[s2].neighbor.push_back(s1);
    }
}




