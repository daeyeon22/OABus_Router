#include "circuit.h"

using namespace OABusRouter;

struct elem
{
    int index;
    int cost;
    int costDest;
    int insertionCount;

    elem() : index(INT_MAX), cost(INT_MAX), costDest(INT_MAX), insertionCount(INT_MAX) {}
    elem(const elem& e)
    {
        index = e.index;
        cost = e.cost;
        costDest = e.costDest;
        insertionCount = e.insertionCount;
    }
};

bool operator > (const elem& left, const elem& right){
    if(left.cost > right.cost) return true;
    else if(left.cost < right.cost) return false;
    if(left.costDest > right.costDest) return true;
    else if(left.costDest < right.costDest) return false;
    if(left.insertionCount > right.insertionCount) return true;
    else return false;
}


Rect OABusRouter::Circuit::get_boundary(Bit* targetBit){
    int llx= INT_MAX, lly = INT_MAX, urx = INT_MIN, ury = INT_MIN;

    for(auto& pinid : targetBit->pins){
        Pin* targetPin = &this->pins[pinid];
        llx = min(targetPin->leftBoundary(), llx);
        lly = min(targetPin->bottomBoundary(), lly);
        urx = max(targetPin->rightBoundary(), urx);
        ury = max(targetPin->topBoundary(), ury);
    }

    return Rect(Point(llx,lly), Point(urx,ury));
}

PairT OABusRouter::Circuit::get_layer_boundary(Bit* targetBit){
    int lower = INT_MAX;
    int upper = INT_MIN;
    for(auto& pinid : targetBit->pins){
        


    }

}

bool OABusRouter::Circuit::route(Bit* targetBit){
 

    typedef pair<bgSegmentT, PairT> LineSegmentT;
    typedef bgi::rtree<LineSegmentT, bgi::rstar<16>> LineSegmentRtreeT;
    vector<LineSegmentT> lineSegments;
    LineSegmentRtreeT lsRtree;

    int lb_layer = INT_MAX, ub_layer = INT_MIN;
    int llx= INT_MAX, lly = INT_MAX, urx = INT_MIN, ury = INT_MIN;

    for(auto& pinid : targetBit->pins){
        Pin* targetPin = &this->pins[pinid];
        llx = min(targetPin->leftBoundary(), llx);
        lly = min(targetPin->bottomBoundary(), lly);
        urx = max(targetPin->rightBoundary(), urx);
        ury = max(targetPin->topBoundary(), ury);
        lb_layer = min(this->layerHashMap[targetPin->layer], lb_layer);
        ub_layer = max(this->layerHashMap[targetPin->layer], ub_layer);
    }

    ub_layer++;

    while(lb_layer < ub_layer){
        Layer* targetLayer = &this->layers[lb_layer++];
        int lowerBound, upperBound;
        if(targetLayer->is_vertical()){
            lowerBound = targetLayer->lower_bound(llx);
            upperBound = targetLayer->upper_bound(urx);
        }else{
            lowerBound = targetLayer->lower_bound(lly);
            upperBound = targetLayer->upper_bound(ury);
        }

        while(lowerBound < upperBound){
            Track* trk = &this->tracks[targetLayer->tracks[lowerBound++]];
            for(auto& emptyInterval : trk->emptyIntervals){
                LineSegmentT ls;
                PairT key(lineSegments.size(), targetLayer->id);
                if(targetLayer->is_vertical()){
                    ls = LineSegmentT(bgSegmentT(bgPointT(trk->offset, emptyInterval.lower()), 
                                bgPointT(trk->offset, emptyInterval.upper())), key);
                }else{
                    ls = LineSegmentT(bgSegmentT(bgPointT(emptyInterval.lower(), trk->offset), 
                                bgPointT(emtpyInterval.upper(), trk->offset)), key);
                }
                
                lineSegments.push_back(ls);
                lsRtree.insert(ls);
            }

        }
    }



}


