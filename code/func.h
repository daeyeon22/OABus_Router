#ifndef __FUNC_H__
#define __FUNC_H__

#include "typedef.h"

namespace OABusRouter
{
    bool intersection(seg s1, seg s2, int &x, int &y);
    bool intersection(seg s1, seg s2, int x1, int y1, int &x2, int &y2);
    void lpt(seg s, int &x, int &y);
    void upt(seg s, int &x, int &y);
    void pts(seg s, int &x1, int &y1, int &x2, int &y2);
    int manhatan_distance(int x1, int y1, int x2, int y2);
    double routing_cost();
    double compactness_cost();
    double segment_cost();
};


#endif
