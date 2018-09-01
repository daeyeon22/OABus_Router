#include "route.h"
#include "circuit.h"
//#define DEBUG_CUT
//#define DEBUG_PATH


void OABusRouter::Circuit::create_path()
{


    int wireid, pinid, x, y;
    bool visit[rou->wires.size()];
    Wire *w1, *w2;
    Pin* curpin;
    MultiPin* curmp;
    memset(visit, false, sizeof(bool) * rou->wires.size());


    for(int i=0; i < buses.size(); i++)
    {
        if(!ckt->buses[i].assign)
            continue;

        int k=0;
        while(true)
        {
            curmp = &multipins[buses[i].multipins[k]];
            if(rou->pin2wire.find(curmp->pins[0]) == rou->pin2wire.end())
                k++;
            else
                break;
        }



        for(int j=0; j < curmp->pins.size(); j++)
        {
            vector<Path> paths;
            stack<int> s;
            curpin = &pins[curmp->pins[j]];

#ifdef DEBUG_PATH
            Wire* w = &rou->wires[rou->pin2wire[curpin->id]];
            printf("%s pin %d (%d %d) (%d %d) -> wire %d (%d %d) (%d %d)\n",
                    curpin->bitName.c_str(), 
                    curpin->id,
                    curpin->llx, curpin->lly, curpin->urx, curpin->ury,
                    w->id, w->x1, w->y1, w->x2, w->y2);

#endif
            
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

                intersection.insert(intersection.end(), w1->intersection.begin(), w1->intersection.end());



                //printf("intersection size : %d\n", intersection.size());
                /*
                for(auto& it : w1->intersection)
                {
                    wireid = it.first;
                    if(wireid != PINTYPE)
                    {
                        intersection.push_back(it);
                    }
                }
                */
                sort(intersection.begin(), intersection.end(), cmp);

                for(auto& it : intersection)
                {

                    x = it.second.first;
                    y = it.second.second;
                    
                    if(it.first == PINTYPE)
                    {
                        pinid = rou->wire2pin[w1->id];
                        if(pins[pinid].l != w1->l)
                        {
                            Path p;
                            p.x[0] = x;
                            p.y[0] = y;
                            p.x[1] = x;
                            p.y[1] = y;
                            p.l = min(pins[pinid].l, w1->l);
                            p.via = true;
                            paths.push_back(p);
                        }
                    }else{
                        //printf("(%d %d) -> %d\n", x,y,it.first);
                        wireid = it.first;
                        w2 = &rou->wires[wireid];
                        if(!visit[wireid])
                        {
                            visit[wireid] = true;
                            s.push(wireid);
                            if(w2->l != w1->l)
                            {
                                Path p;
                                p.x[0] = x;
                                p.x[1] = x;
                                p.y[0] = y;
                                p.y[1] = y;
                                p.l = min(w1->l, w2->l);
                                p.via = true;
                                paths.push_back(p);
                            }
                        }
                    }
                }
            }


            bits[bitHashMap[curpin->bitName]].paths = paths;


#ifdef DEBUG_PATH
            printf("\n\n");
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

