#include "route.h"
#include "circuit.h"

enum Direction
{
    Left = 10,
    Right = 11,
    Up = 12,
    Down = 13,
    Point = 14,
    // target wire routing topologies
    T_Junction = 15,
    EndPointLL = 16,
    EndPointUR = 17
};

struct DirLUT
{
    dense_hash_map<int,int> dir2sign;
    dense_hash_map<int,int> positive;
    dense_hash_map<int,int> negative;
    dense_hash_map<int,int> reverseDir;

    DirLUT()
    {
        dir2sign.set_empty_key(INT_MAX);
        positive.set_empty_key(INT_MAX);
        negative.set_empty_key(INT_MAX);
        reverseDir.set_empty_key(INT_MAX);
    }
    void init();
    int get_strack_direction(int prevSdir, int prevRdir, int curRdir, bool reverse);
};

void DirLUT::init()
{
   int dir[4] = {Direction::Left, Direction::Right, Direction::Down, Direction::Up};
   int sign[4] = { -1, 1, -1, 1 };
   for(int i=0; i < 4 ; i++)
   {
        dir2sign[dir[i]] = sign[i];
        if(i < 2)
        {
            positive[dir[i]] = dir[i+2];
            positive[dir[i+2]] = dir[i];
            negative[dir[i]] = dir[3-i];
            negative[dir[3-i]] = dir[i];
        }
   }

   dir2sign[Direction::Point] = 1;
   reverseDir[dir[0]] = dir[1];
   reverseDir[dir[1]] = dir[0];
   reverseDir[dir[2]] = dir[3];
   reverseDir[dir[3]] = dir[2];
   reverseDir[Direction::Point] = Direction::Point;
   positive[Direction::Point] = Direction::Point;
   negative[Direction::Point] = Direction::Point;
}

int DirLUT::get_strack_direction(int prevRdir, int prevSdir, int curRdir, bool reverse)
{
    int sign = dir2sign[prevSdir] * dir2sign[prevRdir];
    int curSdir = (sign > 0) ? positive[curRdir] : negative[curRdir];
    if(reverse) 
        return reverseDir[curSdir];
    else
        return curSdir;
}


static DirLUT *dirlut = nullptr;




void OABusRouter::Router::move_pt_loc(int &xMoved, int &yMoved, int numBits, int width, int spac, int sDir)
{
    if(sDir == Direction::Left)
    {
        xMoved -= 2*numBits*(width + spac);
    }   
    else if(sDir == Direction::Right)
    {
        xMoved += 2*numBits*(width + spac);
    }
    else if(sDir == Direction::Down)
    {
        yMoved -= 2*numBits*(width + spac);
    }
    else if(sDir == Direction::Up)
    {
        yMoved += 2*numBits*(width + spac);
    }
}

int OABusRouter::Router::get_stack_direction(int m, int p)
{
    Pin* curp = &ckt->pins[p];
    MultiPin* mp = &ckt->multipins[m];

    if(mp->align == VERTICAL)
    {
        if(abs(mp->lly - curp->lly) < abs(mp->ury - curp->ury))
            return Direction::Up;
        else
            return Direction::Down;
    }
    else
    {
        if(abs(mp->llx - curp->llx) < abs(mp->urx - curp->urx))
            return Direction::Right;
        else
            return Direction::Left;
    }
}

int OABusRouter::Router::get_stack_direction(int prevRdir, int prevSdir, int curRdir, bool reverse)
{
    if(dirlut == nullptr)
    {
        dirlut = new DirLUT();
        dirlut->init();
    }
#ifdef DEBUG_TOPOLOGY
    if(prevRdir > 14 || prevRdir < 10)
    {
        cout << "out of range..." << endl;
        cout << "prevRdir : " << prevRdir << endl;
        exit(0);
    }

    if(prevSdir > 14 || prevSdir < 10)
    {
        cout << "out of range...2" << endl;
        cout << "prevSdir : " << prevSdir << endl;
        exit(0);
    }
#endif

    return dirlut->get_strack_direction(prevRdir, prevSdir, curRdir, reverse);
}

bool OABusRouter::Router::get_routing_topology_v3(int numBits, int x[], int y[], int width, int spac, int sDir, polygon &area)
{
    int xMoved[2] = {x[0], x[1]};
    int yMoved[2] = {y[0], y[1]};
    int lenMove = (width + spac) * numBits;
    if(sDir == Direction::Left)
    {
        xMoved[0] -= lenMove;
        xMoved[1] -= lenMove;
    }
    else if(sDir == Direction::Right)
    {
        xMoved[0] += lenMove;
        xMoved[1] += lenMove;
    }
    else if(sDir == Direction::Up)
    {
        yMoved[0] += lenMove;
        yMoved[1] += lenMove;
    }
    else if(sDir == Direction::Down)
    {
        yMoved[0] -= lenMove;
        yMoved[1] -= lenMove;
    }
    typedef PointBG pt; 
    bg::append(area.outer(), pt(x[0], y[0]));
    bg::append(area.outer(), pt(x[1], y[1]));
    bg::append(area.outer(), pt(xMoved[1], yMoved[1]));
    bg::append(area.outer(), pt(xMoved[0], yMoved[0]));

}
bool OABusRouter::Router::get_routing_topology_v2(int numBits, int x[], int y[], int width[], int spac[], int sDir[], polygon &area)
{
    int i;
    int xMoved[2] = {x[0], x[1]};
    int yMoved[2] = {y[0], y[1]};

    int lenMove[2];
    lenMove[0] = (width[0] + spac[0]) * numBits;
    lenMove[1] = (width[1] + spac[1]) * numBits;


    for(i=0; i < 2; i++)
    {
        int curDir = sDir[i];

        if(i == 0)
        {
            if(curDir == Direction::Left)
                xMoved[0] -= lenMove[i];
            else if(curDir == Direction::Right)
                xMoved[0] += lenMove[i];
            else if(curDir == Direction::Up)
                yMoved[0] += lenMove[i];
            else if(curDir == Direction::Down)
                yMoved[0] -= lenMove[i];
            //else
            //    return false;

        }
        else if(i == 1)
        {
            if(curDir == Direction::Left)
            {
                xMoved[0] -= lenMove[i];
                xMoved[1] -= lenMove[i];
            }
            else if(curDir == Direction::Right)
            {
                xMoved[0] += lenMove[i];
                xMoved[1] += lenMove[i];
            }
            else if(curDir == Direction::Up)
            {
                yMoved[0] += lenMove[i];
                yMoved[1] += lenMove[i];
            }
            else if(curDir == Direction::Down)
            {
                yMoved[0] -= lenMove[i];
                yMoved[1] -= lenMove[i];
            }
            //else
            //    return false;
        }

    }


    typedef PointBG pt; 
    bg::append(area.outer(), pt(x[0], y[0]));
    bg::append(area.outer(), pt(x[1], y[1]));
    bg::append(area.outer(), pt(xMoved[1], yMoved[1]));
    bg::append(area.outer(), pt(xMoved[0], yMoved[0]));

    return true;
}
bool OABusRouter::Router::get_routing_topology(int numBits, int x[], int y[], int width[], int spac[], int sDir[], polygon &area)
{
    int i;
    int xMoved[2] = {x[0], x[1]};
    int yMoved[2] = {y[0], y[1]};

    int lenMove[3];
    lenMove[0] = (width[0] + spac[0]) * numBits;
    lenMove[1] = (width[1] + spac[1]) * numBits;
    lenMove[2] = (width[2] + spac[2]) * numBits;


    for(i=0; i < 3; i++)
    {
        int curDir = sDir[i];

        if(i == 0)
        {
            if(curDir == Direction::Left)
                xMoved[0] -= lenMove[i];
            else if(curDir == Direction::Right)
                xMoved[0] += lenMove[i];
            else if(curDir == Direction::Up)
                yMoved[0] += lenMove[i];
            else if(curDir == Direction::Down)
                yMoved[0] -= lenMove[i];
            //else
            //    return false;

        }
        else if(i == 1)
        {
            if(curDir == Direction::Left)
            {
                xMoved[0] -= lenMove[i];
                xMoved[1] -= lenMove[i];
            }
            else if(curDir == Direction::Right)
            {
                xMoved[0] += lenMove[i];
                xMoved[1] += lenMove[i];
            }
            else if(curDir == Direction::Up)
            {
                yMoved[0] += lenMove[i];
                yMoved[1] += lenMove[i];
            }
            else if(curDir == Direction::Down)
            {
                yMoved[0] -= lenMove[i];
                yMoved[1] -= lenMove[i];
            }
            //else
            //    return false;
        }
        else if(i == 2)
        {
            if(curDir == Direction::Left)
                xMoved[1] -= lenMove[i];
            else if(curDir == Direction::Right)
                xMoved[1] += lenMove[i];
            else if(curDir == Direction::Up)
                yMoved[1] += lenMove[i];
            else if(curDir == Direction::Down)
                yMoved[1] -= lenMove[i];
            //else 
            //    return false;
        }
    }

    if(x[0] == x[1])
    {
        if((y[0] > y[1] && yMoved[0] < yMoved[1]) || (y[0] < y[1] && yMoved[0] > yMoved[1]))
        {
            /*
            printf("(%d %d) (%d %d) (%d %d) (%d %d)\n", x[0], y[0], x[1], y[1], xMoved[1], yMoved[1], xMoved[0], yMoved[0]);
            printf("stack direction = { ");
            for(i=0; i < 3; i++)
            {
                if(sDir[i] == Direction::Left)
                    cout << "left "; // << endl;
                if(sDir[i] == Direction::Right)
                    cout << "right "; // << endl;
                if(sDir[i] == Direction::Up)
                    cout << "up "; // << endl;
                if(sDir[i] == Direction::Down)
                    cout << "down "; // << endl;
                if(sDir[i] == Direction::Point)
                    cout << "point "; // << endl;
            }
            cout << "}" << endl;

            for(i=0; i < 3; i++)
                cout << lenMove[i] << endl;
            */
            return false;


        }
    }

    if(y[0] == y[1])
    {
        if((x[0] > x[1] && xMoved[0] < xMoved[1]) || (x[0] < x[1] && xMoved[0] > xMoved[1]))
        {
            /*
            printf("(%d %d) (%d %d) (%d %d) (%d %d)\n", x[0], y[0], x[1], y[1], xMoved[1], yMoved[1], xMoved[0], yMoved[0]);
            printf("stack direction = { ");
            for(i=0; i < 3; i++)
            {
                if(sDir[i] == Direction::Left)
                    cout << "left "; // << endl;
                if(sDir[i] == Direction::Right)
                    cout << "right "; // << endl;
                if(sDir[i] == Direction::Up)
                    cout << "up "; // << endl;
                if(sDir[i] == Direction::Down)
                    cout << "down "; // << endl;
                if(sDir[i] == Direction::Point)
                    cout << "point "; // << endl;
            }
            cout << "}" << endl;
            for(i=0; i < 3; i++)
                cout << lenMove[i] << endl;
            */
            return false;
        }

    }
    typedef PointBG pt; 
    bg::append(area.outer(), pt(x[0], y[0]));
    bg::append(area.outer(), pt(x[1], y[1]));
    bg::append(area.outer(), pt(xMoved[1], yMoved[1]));
    bg::append(area.outer(), pt(xMoved[0], yMoved[0]));

    return true;
}
