#ifndef __FUNC_H__
#define __FUNC_H__

#include "typedef.h"

namespace OABusRouter
{
    
    string direction(int dir);
    bool should_stop();
    bool is_vertical(int l);
    bool intersection(seg s1, seg s2, int &x, int &y);
    bool intersection(seg s1, seg s2, int x1, int y1, int &x2, int &y2);
    void lpt(seg s, int &x, int &y);
    void upt(seg s, int &x, int &y);
    void pts(seg s, int &x1, int &y1, int &x2, int &y2);
    void expand_width(int x[], int y[], int width, bool vertical);
    void into_array(int x1, int x2, int y1, int y2, int x[], int y[]);
    void design_ruled_area(int x[], int y[], int width, int spacing, bool vertical);
    void inverse_vector(vector<int>& target);
    int manhatan_distance(int x1, int y1, int x2, int y2);
    //

};


#endif
