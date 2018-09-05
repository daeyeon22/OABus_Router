#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <set>
#include "func.h"
#include "circuit.h"
#include "route.h"

bool OABusRouter::Router::should_stop()
{
    return ckt->should_stop();
}

void OABusRouter::Router::update_net_tp(vector<Segment> &tp)
{


}




