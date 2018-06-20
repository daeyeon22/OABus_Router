#include "circuit.h"


int OABusRouter::Layer::lower_bound(int coord){
    vector<int>::iterator low = std::lower_bound(this->trackOffsets.begin(), this->trackOffsets.end(), coord);
    return (low - this->trackOffsets.begin();
}

int OABusRouter::Layer::upper_bound(int coord){
    vector<int>::iterator upper = std::upper_bound(this->trackOffsets.begin(), this->trackOffsets.end(), coord);
    return (upper - this->trackOffsets.begin());
}




void OABusRouter::Circuit::initialize(){

    for(auto& lyr : this->layers){
        sort(lyr.tracks.begin(), lyr.tracks.end(), [this](int leftid, int rightid){
                Track* left = &this->tracks[leftid];
                Track* right = &this->tracks[rightid];
                return left->offset < right->offset;
            });

        for(auto& trkid : lyr.tracks){
            Track* trk = &this->tracks[trkid];
            lyr.trackOffsets.push_back(trk->offset);

            if(lyr.direction == VERTICAL){
                trk->emptyIntervals += intervalT::open(trk->ll.y, trk->ur.y);
            }else{
                trk->emptyIntervals += intervalT::open(trk->ll.x, trk->ur.x);
            }
        }
    }


    for(auto& pin : this->pins){
        Layer* targetLayer = &this->layers[this->layerHashMap[pin.layer]];
        int lowerBound, upperBound;
        DiscreteIntervalT pinInterval;
        if(targetLayer->direction == VERTICAL){
            lowerBound = targetLayer->lower_bound(pin.leftBoundary());
            upperBound = targetLayer->upper_bound(pin.rightBoundary());
            pinInterval = DiscreteIntervalT::closed(pin.bottomBoundary(), pin.topBoundary());
        }else{
            lowerBound = targetLayer->lower_bound(pin.bottomBoundary());
            upperBound = targetLayer->upper_bound(pin.topBoundary());
            pinInterval = DiscreteIntervalT::closed(pin.leftBoundary(), pin.rightBoundary());
        }

        while(lowerBound < upperBound){
            Track* trk = &this->layers[targetLayer->tracks[lowerBound++]];
            trk->emptyIntervals -= pinInterval;
        }

    }


    for(auto& obs : this->obstacles){
        Layer* targetLayer = &this->layers[this->layerHashMap[obs.layer]];
        int lowerBound, upperBound;
        DiscreteIntervalT obsInterval;
        if(targetLayer->is_vertical()){
            lower_bound = targetLayer->lower_bound(obs.leftBoundary());
            upper_bound = targetLayer->upper_bound(obs.rightBoundary());
            obsInterval = DiscreteIntervalT::closed(obs.bottomBoundary(), obs.topBoundary());
        }else{
            lowerBound = targetLayer->lower_bound(obs.bottomBoundary());
            upperBound = targetLayer->upper_bound(obs.topBoundary());
            obsInterval = DiscreteIntervalT::closed(obs.leftBoundary(), obs.rightBoundary());
        }

        while(lowerBound < upperBound){
            Track* trk = &this->layers[targetLayer->tracks[lowerBound++]];
            trk->emptyIntervals -= obsInterval;
        }
    }

}



