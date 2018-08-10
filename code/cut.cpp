#include "route.h"
#include "circuit.h"
#define DEBUG_CUT
void OABusRouter::Router::Cut()
{
    
    int i, numwires;
    int trackid, l;
    int x1, y1, x2, y2;
    int offset;
    int xs[2], ys[2];

    Wire* curwire;
    auto cmp = [](const pair<int,int> &left, const pair<int,int> &right)
    {
        return left.first < right.first || left.second < right.second;
    };

    vector<pair<int,int>> intersection;
    numwires = wires.size();
    for(i=0; i < numwires; i++)
    {
        intersection.clear();
        curwire = &wires[i];
           
        if(curwire->via || curwire->intersection.size() < 2) 
            continue;


        printf("orignal (%d %d) (%d %d)\n", curwire->x1, curwire->y1, curwire->x2, curwire->y2);
        for(auto& it : curwire->intersection)
        {
            printf("intersect (%d %d)\n", it.second.first, it.second.second);
            intersection.push_back(it.second);       
        }

        sort(intersection.begin(), intersection.end(), cmp);
        x1 = intersection[0].first;
        x2 = intersection[intersection.size()-1].first;
        y1 = intersection[0].second;
        y2 = intersection[intersection.size()-1].second;
        trackid = curwire->trackid;
        l = curwire->l;
        offset = rtree.track_offset(trackid);

        IntervalSetT iset;
        iset += curwire->vertical ?
            IntervalT::open(curwire->y1, curwire->y2) : IntervalT::open(curwire->x1, curwire->x2);

        iset -= curwire->vertical ?
            IntervalT::open(y1, y2) : IntervalT::open(x1, x2);


        for(auto& it : iset)
        {
            xs[0] = curwire->vertical ? offset : it.lower();
            xs[1] = curwire->vertical ? offset : it.upper();
            ys[0] = curwire->vertical ? it.lower() : offset;
            ys[1] = curwire->vertical ? it.upper() : offset;

            // cut, rtree update
            rtree.insert_element(trackid, xs, ys, l, false);
        }

#ifdef DEBUG_CUT
        printf("changed (%d %d) (%d %d)\n", x1, y1, x2, y2);
#endif
        curwire->x1 = x1;
        curwire->y1 = y1;
        curwire->x2 = x2;
        curwire->y2 = y2;
    }
}



void OABusRouter::Circuit::CreatePath()
{


    int wireid, x, y;
    bool visit[rou->wires.size()];
    Wire *w1, *w2;
    Pin* curpin;
    MultiPin* curmp;
    memset(visit, false, sizeof(bool) * rou->wires.size());
    
  


    for(int i=0; i < buses.size(); i++)
    {
        curmp = &multipins[buses[i].multipins[0]];


        for(int j=0; j < curmp->pins.size(); j++)
        {

            vector<Path> paths;
            stack<int> s;
            curpin = &pins[curmp->pins[j]];
            
            
            s.push(rou->pin2wire[curpin->id]);
            visit[rou->pin2wire[curpin->id]] = true;


            while(!s.empty())
            {
                wireid = s.top();
                s.pop();
                w1 = &rou->wires[wireid];
                if(!w1->via)
                {
                    Path p;
                    p.x[0] = w1->x1;
                    p.x[1] = w1->x2;
                    p.y[0] = w1->y1;
                    p.y[1] = w1->y2;
                    p.l = w1->l;
                    p.via = false;
                    paths.push_back(p);
                }



                vector<pair<int,pair<int,int>>> intersection;
                
                auto cmp = [](const pair<int, pair<int,int>> &left, const pair<int, pair<int,int>> &right)
                {
                    return left.second.first < right.second.first || left.second.second < right.second.second;
                };

                for(auto& it : w1->intersection)
                {
                    wireid = it.first;
                    if(wireid != PINTYPE)
                    {
                        intersection.push_back(it);
                    }
                }
                
                sort(intersection.begin(), intersection.end(), cmp);

                for(auto& it : intersection)
                {
                    wireid = it.first;
                    w2 = &rou->wires[wireid];
                    x = it.second.first;
                    y = it.second.second;
                    if(w1->l == w2->l) continue;

                    if(!visit[wireid])
                    {
                        
                        visit[wireid] = true;
                        Path p;
                        p.x[0] = x;
                        p.x[1] = x;
                        p.y[0] = y;
                        p.y[1] = y;
                        p.l = min(w1->l, w2->l);
                        p.via = true;
                        s.push(wireid);
                        paths.push_back(p);
                    }
                }
            }


            bits[bitHashMap[curpin->bitName]].paths = paths;


#ifdef DEBUG_CUT
            printf("BIT %s\n", curpin->bitName.c_str());
            printf("PATH %d\n", paths.size());
            for(auto& p : paths)
            {
                if(p.via)
                    printf("%s (%d %d)\n", layers[p.l].name.c_str(), p.x[0], p.y[0]);
                else
                    printf("%s (%d %d) (%d %d)\n",layers[p.l].name.c_str(), p.x[0], p.y[0], p.x[1], p.y[1]);
            }
            printf("END PATH\n");
            printf("END BIT\n");
            printf("\n\n");
#endif
        }
    }


}

