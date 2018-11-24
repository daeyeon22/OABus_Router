#include "func.h"
#include "circuit.h"
#include "route.h"
#include "mymeasure.h"

void OABusRouter::inverse_vector(vector<int> &target)
{
    vector<int> flipped;
    for(int i=0; i < target.size(); i++)
    {
        flipped.insert(flipped.begin(), target[i]);
    }

    target = flipped;
}

string OABusRouter::direction(int dir)
{
    switch(dir)
    {
        case Direction::Left:
            return "Left";      
            break;
        case Direction::Right:
            return "Right";
            break;
        case Direction::Up:
            return "Up";
            break;
        case Direction::Down:
            return "Down";
            break;

        case Direction::Stack:
            return "Stack";
            break;
        default:
            return "";
            break;
    }
}


bool OABusRouter::should_stop()
{
    double elapse_time = measure.elapse_time();
    double runtime_limit = (double)ckt->runtime * 60;


    if(elapse_time > 0.9*runtime_limit)
    {
#ifdef REPORT
        printf("\n");
        printf("[INFO] elapse time over 80% runtime limit\n");
        printf("[INFO] Runtime limit   : %.2f\n", runtime_limit);
        printf("[INFO] Elapse time     : %.2f\n", elapse_time);
        printf("\n");
#endif
        return false;
        //return true;
    }
    else
        return false;
}



void OABusRouter::design_ruled_area(int x[], int y[], int width, int spac, bool vertical)
{
    if(vertical)
    {
        x[0] -= ((int)(1.0*width / 2) + spac);
        x[1] += ((int)(1.0*width / 2) + spac);
        y[0] -= spac;
        y[1] += spac;
    }
    else
    {
        y[0] -= ((int)(1.0*width / 2) + spac);
        y[1] += ((int)(1.0*width / 2) + spac);
        x[0] -= spac;
        x[1] += spac;
    }
}


void OABusRouter::into_array(int x1, int x2, int y1, int y2, int x[], int y[])
{
    x[0] = x1;
    x[1] = x2;
    y[0] = y1;
    y[1] = y2;
}

void OABusRouter::expand_width(int x[], int y[], int width, bool vertical)
{
    if(vertical)
    {
        x[0] -= width/2;
        x[1] += width/2;
    }
    else
    {
        y[0] -= width/2;
        y[1] += width/2;
    }
}


bool OABusRouter::is_vertical(int l)
{
    return (ckt->layers[l].direction == VERTICAL) ? true : false; 
}


bool OABusRouter::intersection(seg s1, seg s2, int &x, int &y)
{
    using OABusRouter::pts;


    if(!bg::intersects(s1, s2))
    {
        return false;
    }

    int llx1, lly1, urx1, ury1;
    int llx2, lly2, urx2, ury2;
    bool vertical1, vertical2;
    pts(s1, llx1, lly1, urx1, ury1);
    pts(s2, llx2, lly2, urx2, ury2);
    vertical1 = (llx1 == urx1) ? true : false;
    vertical2 = (llx2 == urx2) ? true : false;

    bool valid = false;
    if(vertical1 && !vertical2)
    {
        if((lly1 <= lly2 && lly2 <= ury1) && (llx2 <= llx1 && llx1 <= urx2))
        {
            x = llx1;
            y = lly2;
            valid = true;
        }
    }
    else if(!vertical1 && vertical2)
    {
        if((lly2 <= lly1 && lly1 <= ury2) && (llx1 <= llx2 && llx2 <= urx1))
        {
            x = llx2;
            y = lly1;
            valid = true;
        }
    }
    else
    {
        if(llx1 == urx2 && lly1 == ury2)
        {
            x = llx1;
            y = lly1;
            valid = true;
        }
        else if(urx1 == llx2 && ury1 == lly2)
        {
            x = urx1;
            y = ury1;
            valid = true;
        }


    }

    return valid;
}

bool OABusRouter::intersection(seg s1, seg s2, int x1, int y1, int &x2, int &y2)
{
    using OABusRouter::pts;

    int llx1, lly1, urx1, ury1;
    int llx2, lly2, urx2, ury2;
    bool vertical1, vertical2;
    pts(s1, llx1, lly1, urx1, ury1);
    pts(s2, llx2, lly2, urx2, ury2);
    vertical1 = (llx1 == urx1) ? true : false;
    vertical2 = (llx2 == urx2) ? true : false;

    bool valid = false;
    if(vertical1 && !vertical2)
    {
        x2 = llx1;
        y2 = lly2;
        valid = true;
    }
    else if(!vertical1 && vertical2)
    {
        x2 = llx2;
        y2 = lly1;
        valid = true;
    }
    else
    {
        if(llx1 == urx2 && lly1 == ury2)
        {
            x2 = llx1;
            y2 = lly1;
            valid = true;
        }
        else if(urx1 == llx2 && ury1 == lly2)
        {
            x2 = urx1;
            y2 = ury1;
            valid = true;
        }
        else
        {
            using namespace std;

            if(vertical1)
            {
                if(y1 < lly2)
                {
                    x2 = llx2;
                    y2 = lly2;
                    valid = true;
                }
                else if(y1 > ury2)
                {
                    x2 = urx2;
                    y2 = ury2;
                    valid = true;
                }
                else
                {
                    x2 = x1;
                    y2 = y1;
                    valid = true;
                }
            }
            else
            {
                if(x1 < llx2)
                {
                    x2 = llx2;
                    y2 = lly2;
                    valid = true;
                }
                else if(x1 > urx2)
                {
                    x2 = urx2;
                    y2 = ury2;
                    valid = true;
                }
                else
                {
                    x2 = x1;
                    y2 = y1;
                    valid = true;
                }
            }
        }
    }

    return valid;
}

void OABusRouter::lpt(seg s, int &x, int &y)
{
    x = (int)(bg::get<0,0>(s) + 0.5);
    y = (int)(bg::get<0,1>(s) + 0.5);
}

void OABusRouter::upt(seg s, int &x, int &y)
{
    x = (int)(bg::get<1,0>(s) + 0.5);
    y = (int)(bg::get<1,1>(s) + 0.5);
}

void OABusRouter::pts(seg s, int &x1, int &y1, int &x2, int &y2)
{
    x1 = (int)(bg::get<0,0>(s) + 0.5);
    y1 = (int)(bg::get<0,1>(s) + 0.5);
    x2 = (int)(bg::get<1,0>(s) + 0.5);
    y2 = (int)(bg::get<1,1>(s) + 0.5);
}

int OABusRouter::manhatan_distance(int x1, int y1, int x2, int y2)
{
    return abs(x1-x2) + abs(y1-y2);
}


