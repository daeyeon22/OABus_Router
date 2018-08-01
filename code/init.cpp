#include "func.h"
#include "circuit.h"
#include <tuple>

using namespace OABusRouter;



void OABusRouter::Circuit::Init()
{
    Getpitch();
}

void OABusRouter::Circuit::Getpitch()
{
    int i, j;
    int minpitch, maxpitch;
    int numl, numos;
    int diff;

    numl = layers.size();
    Layer* curL;
    for(i=0; i < numl; i++)
    {
        curL = &layers[i];
        minpitch = INT_MAX;
        maxpitch = INT_MIN;

        numos = curL->trackOffsets.size() -1;
        for(j=0; j < numos; j++)
        {
            diff = abs(curL->trackOffsets[j] - curL->trackOffsets[j+1]);
            if(diff == 0) continue;

            minpitch = min(minpitch, diff);
            maxpitch = max(maxpitch, diff);
        }

        curL->minpitch = minpitch;
        curL->maxpitch = maxpitch;
    }

}



