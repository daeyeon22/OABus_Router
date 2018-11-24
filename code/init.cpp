#include "circuit.h"
#include "route.h"
#include "typedef.h"
#include "func.h"
#include <tuple>

using namespace OABusRouter;
using OABusRouter::intersection;
using OABusRouter::is_vertical;



void OABusRouter::Circuit::remove_redundant_tracks()
{
    namespace bi = boost::icl;

    int numLayers = layers.size();
    vector<Track> newTracks; 

    cout << "[INFO] Remove redundant tracks" << endl;
    cout << "[INFO] Before #Tracks " << tracks.size() << endl;

    for(int i=0; i < numLayers; i++)
    {
        //set<int> done;
        //vector<Track> tmp;
        Layer* curLayer = &layers[i];

        vector<int> offsets;
        vector<int> types;
        dense_hash_map<int,vector<int>> offset2tracks;
        offset2tracks.set_empty_key(INT_MAX);

        for(int j=0; j < curLayer->tracks.size(); j++)
        {
            Track* curT = &tracks[curLayer->tracks[j]];
            offset2tracks[curT->offset].push_back(curT->id);
            //
            if(find(types.begin(), types.end(), curT->width) == types.end())
                types.push_back(curT->width);

            //
            if(find(offsets.begin(), offsets.end(), curT->offset) == offsets.end())
                offsets.push_back(curT->offset);
        }

        sort(types.begin(), types.end(), [](int left, int right){
            return left > right;
                });
       
        curLayer->tracks.clear();
        #pragma omp parallel for num_threads(NUM_THREADS)
        for(int j=0; j < offsets.size(); j++)
        {


            int offset = offsets[j];


        /*
            dense_hash_map<int, bi::interval_set<int>> width2interval;
            width2interval.set_empty_key(INT_MAX);

            for(auto& trackid : offset2tracks[offset])
            {
                Track* curT = &tracks[trackid];
                
                if(curLayer->direction == VERTICAL)
                    width2interval[curT->width] += bi::interval<int>::closed(curT->lly, curT->ury);
                else
                    width2interval[curT->width] += bi::interval<int>::closed(curT->llx, curT->urx);
            }

            for(int k=0; k < types.size(); k++)
            {
                bi::interval_set<int> curSet = width2interval[types[k]];

                bi::interval_set<int>::iterator it = curSet.begin();
                while(it != curSet.end())
                {
                    Track newTrack;
                    //newTrack.id = tmp.size();
                    newTrack.width = types[k];
                    newTrack.offset = offsets[j];
                    newTrack.llx = is_vertical(curLayer->id) ? offset : it->lower();
                    newTrack.urx = is_vertical(curLayer->id) ? offset : it->upper();
                    newTrack.lly = is_vertical(curLayer->id) ? it->lower() : offset;
                    newTrack.ury = is_vertical(curLayer->id) ? it->upper() : offset;
                    newTrack.l = curLayer->id; 
                    #pragma omp critical(GLOBAL)
                    {
                        newTrack.id = newTracks.size();
                        curLayer->tracks.push_back(newTrack.id);
                        newTracks.push_back(newTrack);
                    }
                    it++;
                }

                
                for(int l=k+1; l < offsets.size(); l++)
                {
                    width2interval[types[l]] -= curSet;
                }
                
            }
        }
        */

        
            bi::interval_set<int> interval;
            bi::interval_map<int,set<int>> constraints; //widthConsts;

            for(auto& trackid : offset2tracks[offset])
            {
                Track* curT = &tracks[trackid];
                
                if(curLayer->direction == VERTICAL)
                {
                    interval += bi::interval<int>::closed(curT->lly, curT->ury);
                    constraints += make_pair(bi::interval<int>::closed(curT->lly, curT->ury), set<int>({curT->width}));
                }
                else
                {
                    interval += bi::interval<int>::closed(curT->llx, curT->urx);
                    constraints += make_pair(bi::interval<int>::closed(curT->llx, curT->urx), set<int>({curT->width}));
                }
            }
            
            bi::interval_set<int>::iterator it = interval.begin();
            while(it != interval.end())
            {
                Track newTrack;
                //newTrack.id = tmp.size();
                newTrack.width = types[0];
                newTrack.offset = offsets[j];
                newTrack.llx = is_vertical(curLayer->id) ? offset : it->lower();
                newTrack.urx = is_vertical(curLayer->id) ? offset : it->upper();
                newTrack.lly = is_vertical(curLayer->id) ? it->lower() : offset;
                newTrack.ury = is_vertical(curLayer->id) ? it->upper() : offset;
                newTrack.l = curLayer->id; 
               
                /* 
                for(auto& itPair : it & constraints)
                {
                    
                    
                    if(bi::intersects(it, itPair.fisrst))
                    {
                        vector<int> widths;
                        for(auto& w : itPair.second)
                        {
                            
                            maxWidth = max(maxWidth, w);
                        }
                        newTrack.constraints += make_pair(itPair.first, maxWidth); 
                    }
                }
                */

                #pragma omp critical(GLOBAL)
                {
                    newTrack.id = newTracks.size();
                    curLayer->tracks.push_back(newTrack.id);
                    newTracks.push_back(newTrack);
                }
                it++;

                
            }

            /*///////////////////////////////////////////////////////////////////////////////////
            bi::interval_set<int> interval_empty;
            if(curLayer->direction == VERTICAL)
                interval_empty += bi::interval<int>::open(ckt->originY, ckt->originY + ckt->height);
            else
                interval_empty += bi::interval<int>::open(ckt->originX, ckt->originX + ckt->width);

            interval_empty -= interval;

            it = interval_empty.begin();
            while(it != interval_empty.end())
            {
                Empty newEmpty;
                newEmpty.offset = offsets[j];
                newEmpty.llx = is_vertical(curLayer->id) ? offset : it->lower();
                newEmpty.urx = is_vertical(curLayer->id) ? offset : it->upper();
                newEmpty.lly = is_vertical(curLayer->id) ? it->lower() : offset;
                newEmpty.ury = is_vertical(curLayer->id) ? it->upper() : offset;
                newEmpty.l = curLayer->id; 
                #pragma omp critical(GLOBAL)
                {
                    newEmpty.id = empties.size();
                    empties.push_back(newEmpty);
                }
                it++;
            }
            *////////////////////////////////////////////////////////////////////////////////////

        }
        //*/


        /*
        #pragma omp critical(GLOBAL)
        {
            for(int j=0; j < tmp.size(); j++)
            {
                tmp[j].id = newTracks.size(); 
                curLayer->tracks.push_back(tmp[j].id);
                newTracks.push_back(tmp[j]);
            }
        }
        

        sort(tmp.begin(), tmp.end(), [&, this, curLayer](int left, int right){
                Track* t1 = &this->tracks[left];
                Track* t2 = &this->tracks[right];
                int length1 = manhatan_distance(t1->llx, t1->lly, t1->urx, t1->ury);
                int length2 = manhatan_distance(t2->llx, t2->lly, t2->urx, t2->ury);
                if(curLayer->direction == VERTICAL)
                    return (t1->lly < t2->lly) || ((t1->lly == t2->lly) && (length1 >= length2));
                else
                    return (t1->llx < t2->llx) || ((t1->llx == t2->llx) && (length1 >= length2));
                });


        for(int a=0; a < tmp.size(); a++)
        {
            Track* t1 = &tracks[tmp[a]];

            if(done.find(t1->id) != done.end())
                continue;

            
            Track newTrack(*t1);
            done.insert(t1->id);

            if(a != tmp.size()-1)
            {
                for(int b=a+1; b < tmp.size(); b++)
                {
                    Track* t2 = &tracks[tmp[b]];
                    
                    if(done.find(t2->id) != done.end())
                        continue;

                    if(curLayer->direction == VERTICAL)
                    {
                        if(t1->lly != t2->lly)
                            break;
                    }
                    else
                    {
                        if(t1->llx != t2->llx)
                            break;
                    }

                    if(t1->offset != t2->offset)
                        continue;

                    if(curLayer->direction == VERTICAL)
                    {
                        if(t1->ury >= t2->ury)
                        {
                            newTrack.width = max(newTrack.width, t2->width);
                            done.insert(t2->id);
                        }
                    }
                    else
                    {
                        if(t1->urx >= t2->urx)
                        {
                            newTrack.width = max(newTrack.width, t2->width);
                            done.insert(t2->id);
                        }
                    }
                }
            }

            #pragma omp critical(GLOBAL)
            {
                newTrack.id = newTracks.size();
                newTracks.push_back(newTrack);
                curLayer->tracks.push_back(newTrack.id);
            }
        }
        */
    }

    tracks.clear();
    tracks.insert(tracks.end(), newTracks.begin(), newTracks.end());

    cout << "[INFO] After #Tracks " << tracks.size() << endl;

#ifdef DEBUG_INIT
    printf("\n- - - - <Track Info> - - - -\n");
    for(int i=0; i < tracks.size(); i++)
    {
        Track* curT = &tracks[i];
        printf("t%d (%d %d) (%d %d) M%d width %d\n", curT->id, curT->llx, curT->lly, curT->urx, curT->ury, curT->l, curT->width);
    }
    printf("\n");
#endif

}


void OABusRouter::Circuit::initialize()
{
    int i, j, k, l, align;
    int numpins, numbits, numbuses, numlayers;
    Bus* curbus;
    Bit* curbit;
    Pin* curpin;
    numbuses = buses.size();
    numlayers = layers.size();
    numpins = pins.size();


    // spacing
    for(i=0; i < numlayers; i++)
        rou->spacing[i] = layers[i].spacing;
    
    // mapping
    //#pragma omp parallel for num_threads(NUM_THREADS)
    for(i=0; i < numpins ; i++)
    {
        curpin = &pins[i];
        curbit = &bits[bitHashMap[curpin->bitName]];
        curbus = &buses[busHashMap[curbit->busName]];
        rou->pin2bit[curpin->id] = curbit->id;
        rou->pin2bus[curpin->id] = curbus->id;
        //bitHashMap[curpin->bitName];
    }

    //#pragma omp parallel for num_threads(NUM_THREADS)
    for(i=0; i < numbuses; i++)
    {
        curbus = &buses[i];
        numpins = curbus->numPinShapes;
        numbits = curbus->numBits;
        vector<vector<int>> unused_pins(numbits);

        for(j=0; j < numbits; j++)
        {
            // added
            rou->bit2bus[curbus->bits[j]] = curbus->id;

            unused_pins[j] = bits[curbus->bits[j]].pins;
        }

        for(j=0; j < numpins; j++)
        {
            MultiPin mp;
            mp.id = multipins.size();
            mp.busid = curbus->id;

            for(k=0; k < numbits; k++)
            {
                curbit = &bits[curbus->bits[k]];
                curpin = &pins[curbit->pins[j]];
                //
                //printf("p%d -> bitid %d\n", curpin->id, curbit->id);
                //rou->pin2bit[curpin->id] = curbit->id;
                if(k==0)
                {
                    mp.l = curpin->l;
                    mp.llx = curpin->llx;
                    mp.lly = curpin->lly;
                    mp.urx = curpin->urx;
                    mp.ury = curpin->ury;
                    mp.pins.push_back(curpin->id);
                }
                else
                {
                    vector<int> &sorted = unused_pins[k];
                    box mpbb(pt(mp.llx, mp.lly), pt(mp.urx, mp.ury));
                    l = mp.l;
                    if(sorted.size() > 1)
                    {
                        sort(sorted.begin(), sorted.end(), [&,this, mpbb,l](int left, int right){
                                Pin* p1 = &this->pins[left];
                                Pin* p2 = &this->pins[right];
                                float dist1 = (p1->l != l) ? FLT_MAX : bg::distance(mpbb, box(pt(p1->llx,p1->lly), pt(p1->urx, p1->ury)));
                                float dist2 = (p2->l != l) ? FLT_MAX : bg::distance(mpbb, box(pt(p2->llx,p2->lly), pt(p2->urx, p2->ury))); 
                                return dist1 < dist2;
                                });
                    }
                    curpin = &pins[sorted[0]];
                    mp.llx = min(curpin->llx, mp.llx); 
                    mp.lly = min(curpin->lly, mp.lly); 
                    mp.urx = max(curpin->urx, mp.urx); 
                    mp.ury = max(curpin->ury, mp.ury); 
                    mp.pins.push_back(sorted[0]);
                    sorted.erase(sorted.begin());
                }
            }
            
            if(abs(mp.urx - mp.llx) > abs(mp.ury - mp.lly))
                mp.align = HORIZONTAL;
            else
                mp.align = VERTICAL;

            if(mp.align == layers[mp.l].direction)
                mp.needVia = true;

            /*
            align = mp.align;
            sort(mp.pins.begin(), mp.pins.end(), [&,this,align](int left, int right){
                if(align == VERTICAL)
                    return this->pins[left].lly < this->pins[right].lly;
                else

                    });
            */

            #pragma omp critical(GLOBAL)
            {
                curbus->multipins.push_back(mp.id);
                multipins.push_back(mp);
            }
            //
            rou->multipin2llx[mp.id] = mp.llx;
            rou->multipin2lly[mp.id] = mp.lly;
            rou->multipin2urx[mp.id] = mp.urx;
            rou->multipin2ury[mp.id] = mp.ury;
        }
    }

    for(i=0; i < multipins.size(); i++)
    {
        MultiPin* mp = &multipins[i];
        for(j=0; j < mp->pins.size(); j++)
            rou->pin2align[mp->pins[j]] = mp->align;
    }


    //rou->DEPTH_COST = (height + width)*5/100;
    //rou->VIA_COST = (height + width)*10/1000;
    //rou->SPACING_VIOLATION = (height + width)*50/100;
    //rou->NOTCOMPACT = (height + width)/5;
}


void OABusRouter::Router::construct_rtree()
{
    
    int numTracks, numLayers, trackid;
    numLayers = ckt->layers.size();
    vector<Track> newTracks; 
    vector<RtreeNode> newNodes;
    vector<Constraint> newConsts;

    //vector<PointRtree> ptrtree(numLayers);
    rtree_t.trees = vector<SegRtree>(numLayers);
    
    cout << "[INFO] Remove redundant tracks" << endl;
    cout << "[INFO] Before #Tracks " << ckt->tracks.size() << endl;

    for(int i=0; i < numLayers; i++)
    {
        //set<int> done;
        //vector<Track> tmp;
        Layer* curLayer = &ckt->layers[i];

        vector<int> offsets;
        vector<int> types;
        dense_hash_map<int,vector<int>> offset2tracks;
        offset2tracks.set_empty_key(INT_MAX);

        for(int j=0; j < curLayer->tracks.size(); j++)
        {
            Track* curT = &ckt->tracks[curLayer->tracks[j]];
            offset2tracks[curT->offset].push_back(curT->id);
            //
            if(find(types.begin(), types.end(), curT->width) == types.end())
                types.push_back(curT->width);

            //
            if(find(offsets.begin(), offsets.end(), curT->offset) == offsets.end())
                offsets.push_back(curT->offset);
        }

        sort(types.begin(), types.end(), [](int left, int right){
            return left > right;
                });
       
        curLayer->tracks.clear();
        #pragma omp parallel for num_threads(NUM_THREADS)
        for(int j=0; j < offsets.size(); j++)
        {
            int offset = offsets[j];
       
            bi::interval_set<int> interval;
            bi::interval_map<int,set<int>> constraints; //widthConsts;

            for(auto& trackid : offset2tracks[offset])
            {
                Track* curT = &ckt->tracks[trackid];
                
                if(curLayer->direction == VERTICAL)
                {
                    interval += bi::interval<int>::closed(curT->lly, curT->ury);
                    constraints += make_pair(bi::interval<int>::closed(curT->lly, curT->ury), set<int>({curT->width}));
                }
                else
                {
                    interval += bi::interval<int>::closed(curT->llx, curT->urx);
                    constraints += make_pair(bi::interval<int>::closed(curT->llx, curT->urx), set<int>({curT->width}));
                }
            }
            
            bi::interval_set<int>::iterator it = interval.begin();
            while(it != interval.end())
            {
                Track newTrack;
                //newTrack.id = tmp.size();
                newTrack.width = types[0];
                newTrack.offset = offsets[j];
                newTrack.llx = is_vertical(curLayer->id) ? offset : it->lower();
                newTrack.urx = is_vertical(curLayer->id) ? offset : it->upper();
                newTrack.lly = is_vertical(curLayer->id) ? it->lower() : offset;
                newTrack.ury = is_vertical(curLayer->id) ? it->upper() : offset;
                newTrack.l = curLayer->id; 
               
                RtreeNode newNode; // = rtree_t.get_node(i);
                newNode.offset = offsets[j];
                newNode.x1 = is_vertical(curLayer->id) ? offset : it->lower();
                newNode.x2 = is_vertical(curLayer->id) ? offset : it->upper();
                newNode.y1 = is_vertical(curLayer->id) ? it->lower() : offset;
                newNode.y2 = is_vertical(curLayer->id) ? it->upper() : offset;
                newNode.l = curLayer->id; 
                newNode.vertical = is_vertical(curLayer->id);
                newNode.width = types[0];
                seg s1(pt(newNode.x1, newNode.y1), pt(newNode.x2, newNode.y2));
                //pt p1((n1->x1 + n1->x2)/2, (n1->y1 + n1->y2)/2);
                

                Constraint newConst;
                for(auto& itPair : constraints)
                {
                    int lower = max(it->lower(), itPair.first.lower()); //itPair.first).lower();
                    int upper = min(it->upper(), itPair.first.upper()); //(it & itPair.first).upper();
                    if(lower >= upper) 
                        continue;
                    
                    
                    for(auto& w : itPair.second)
                    {
                        newConst.min_width = min(newConst.min_width, w);
                        newConst.max_width = max(newConst.max_width, w);
                        if(find(newConst.widths.begin(), newConst.widths.end(), w) == newConst.widths.end())
                        {
                            newConst.widths.push_back(w);
                        }
                        newConst.constraint[w] += bi::interval<int>::closed(lower, upper); 
                    }
                }

                #pragma omp critical(GLOBAL)
                {
                    int id = newTracks.size();
                    int l = curLayer->id;
                    newTrack.id = id;
                    newNode.id = id;
                    newConsts.push_back(newConst);
                    newTracks.push_back(newTrack);
                    newNodes.push_back(newNode);
                    
                    curLayer->tracks.push_back(id);
                    rtree_t[l]->insert({s1, id});
                    //ptrtree[l].insert({p1, id});

                }
                it++;
            }

            bi::interval_set<int> interval_empty;
            if(curLayer->direction == VERTICAL)
                interval_empty += bi::interval<int>::open(ckt->originY, ckt->originY + ckt->height);
            else
                interval_empty += bi::interval<int>::open(ckt->originX, ckt->originX + ckt->width);

            interval_empty -= interval;

            it = interval_empty.begin();
            while(it != interval_empty.end())
            {
                Empty newEmpty;
                newEmpty.offset = offsets[j];
                newEmpty.llx = is_vertical(curLayer->id) ? offset : it->lower();
                newEmpty.urx = is_vertical(curLayer->id) ? offset : it->upper();
                newEmpty.lly = is_vertical(curLayer->id) ? it->lower() : offset;
                newEmpty.ury = is_vertical(curLayer->id) ? it->upper() : offset;
                newEmpty.l = curLayer->id; 
                #pragma omp critical(GLOBAL)
                {
                    newEmpty.id = ckt->empties.size();
                    ckt->empties.push_back(newEmpty);
                }
                it++;
            }

        }
        //*/

    }

    ckt->tracks.clear();
    ckt->tracks = newTracks; //.insert(tracks.end(), newTracks.begin(), newTracks.end());

    cout << "[INFO] After #Tracks " << ckt->tracks.size() << endl;
/*
#ifdef DEBUG_INIT
    printf("\n- - - - <Track Info> - - - -\n");
    for(int i=0; i < ckt->tracks.size(); i++)
    {
        Track* curT = &ckt->tracks[i];
        printf("t%d (%d %d) (%d %d) M%d width %d\n", curT->id, curT->llx, curT->lly, curT->urx, curT->ury, curT->l, curT->width);
    }
    printf("\n");
#endif
*/
    numTracks = ckt->tracks.size();
    numLayers = ckt->layers.size();
    //rtree_t.nodes = vector<RtreeNode>(numTracks);
    rtree_t.nodes = newNodes;
    rtree_t.constraints = newConsts;

    newNodes.clear();
    newTracks.clear();
    newConsts.clear();

    set<pair<int,int>> edges;
    vector<SegRtree> rtreeTemp = rtree_t.trees;
    /*
    #pragma omp parallel for num_threads(NUM_THREADS)
    for(int i=0; i < numTracks; i++)
    {
        RtreeNode* n1 = rtree_t.get_node(i);
        seg s1(pt(n1->x1, n1->y1), pt(n1->x2, n1->y2));
        
        #pragma omp critical(TREE)
        {
            rtreeTemp[n1->l].remove({s1, n1->id});
        }

        vector<pair<seg,int>> queries;
        for(int j=n1->l; j <= min(numLayers-1, n1->l+1); j++)
        {
            #pragma omp critical(TREE)
            {
                rtreeTemp[j].query(bgi::intersects(s1), back_inserter(queries));
            }
        }

        for(int j=0; j < queries.size(); j++)
        {
            int x, y;
            RtreeNode* n2 = rtree_t.get_node(queries[j].second);
            seg s2 = queries[j].first;
            
            if(n1->id == n2->id)
                continue;
        
            if(!rtree_t.intersection(n1->id, n2->id, x, y))
                continue;


            #pragma omp critical(EDGE)
            {
                pair<int,int> e(min(n1->id, n2->id), max(n1->id, n2->id));
                if(edges.find(e) == edges.end())
                {
                    n1->intersection[n2->id] = {x, y};
                    n2->intersection[n1->id] = {x, y};
                    edges.insert(e);
                }
            }
        }
    }
    */

    cout << "[INFO] Create nodes " << rtree_t.nodes.size() << endl;
    /*
    #pragma omp parallel for num_threads(NUM_THREADS)
    for(int i=0; i < numTracks; i++)
    {
        RtreeNode* n1 = rtree_t.get_node(i);
        seg s1(pt(n1->x1, n1->y1), pt(n1->x2, n1->y2));

        vector<pair<seg,int>> queries;
        for(int j=n1->l; j <= min(numLayers-1, n1->l+1); j++)
        {
            rtree_t[j]->query(bgi::intersects(s1), back_inserter(queries));
        }

        for(int j=0; j < queries.size(); j++)
        {
            int x, y;
            RtreeNode* n2 = rtree_t.get_node(queries[j].second);
            seg s2 = queries[j].first;
            
            if(n1->id == n2->id)
                continue;

            
            ////////////
            if(!bg::intersects(s1, s2))
            {
                //cout << "??? not intersects..." << endl;
                //printf("t1 (%d %d) (%d %d) M%d width %d\n", n1->x1, n1->y1, n1->x2, n1->y2, n1->l, n1->width);
                //printf("t2 (%d %d) (%d %d) M%d width %d\n", n2->x1, n2->y1, n2->x2, n2->y2, n2->l, n2->width);
            }



            if(!intersection(s1, s2, x, y))
            {
                //cout << "Not intersection" << endl;
                //printf("t1 (%d %d) (%d %d) M%d width %d\n", n1->x1, n1->y1, n1->x2, n1->y2, n1->l, n1->width);
                //printf("t2 (%d %d) (%d %d) M%d width %d\n", n2->x1, n2->y1, n2->x2, n2->y2, n2->l, n2->width);
                continue;
            }

            
            if(n1->l == n2->l)
            {
                if(x == n1->x1 && y == n1->y1)
                    n1->prev = n2;
                else if(x == n1->x2 && y == n1->y2)
                    n1->next = n2;
                else
                {
                    cout << "Same layer " << x << " " << y <<  endl;
                    cout << n1->id << " " << bg::dsv(s1) << endl;
                    cout << n2->id << " " << bg::dsv(s2) << endl << endl;

                }
            }

            #pragma omp critical(GLOBAL)
            {
                if(n1->intersection.find(n2->id) == n1->intersection.end())
                {
                    n1->neighbor.push_back(n2->id);
                    n2->neighbor.push_back(n1->id);
                    n1->intersection[n2->id] = {x,y};
                    n2->intersection[n1->id] = {x,y};
                }
                rtree_t.edges.insert( {min(n1->id, n2->id), max(n1->id, n2->id)} );
                //printf("edge (%d %d)\n", min(n1->id, n2->id), max(n1->id, n2->id)); 
                //printf("t1 (%d %d) (%d %d) M%d width %d\n", n1->x1, n1->y1, n1->x2, n1->y2, n1->l, n1->width);
                //printf("t2 (%d %d) (%d %d) M%d width %d\n", n2->x1, n2->y1, n2->x2, n2->y2, n2->l, n2->width);
            }
        }
    }
    */

    #pragma omp parallel for num_threads(NUM_THREADS) 
    for(int i=0; i < numTracks; i++)
    {
        int l, x1, y1, x2, y2, offset1, offset2;
        RtreeNode *n1, *n2;
        n1 = rtree_t.get_node(i);
        l = n1->l;
        
        seg s1(pt(n1->x1, n1->y1), pt(n1->x2, n1->y2));
        offset1 = n1->offset;

        for(SegRtree::const_query_iterator it = rtree_t.trees[l].qbegin(bgi::nearest(s1, 100)); it != rtree_t.trees[l].qend(); it++)
        {
            if(n1->upper != nullptr && n1->lower != nullptr)
                break;

            n2 = rtree_t.get_node(it->second);
            offset2 = n2->offset;

            if(offset1 == offset2)
                continue;
            
            if(offset1 < offset2)
            {
                if(n1->upper == nullptr)
                    n1->upper = n2;
            }
            else if(offset1 > offset2)
            {
                if(n1->lower == nullptr)
                    n1->lower = n2;
            }
        }

        /*
        x1 = (n1->x1 + n1->x2)/2;
        y1 = (n1->y1 + n1->y2)/2;

        offset1 = n1->vertical ? x1 : y1;

        for(PointRtree::const_query_iterator it = ptrtree[l].qbegin(bgi::nearest(pt(x1,y1), 10)); it != ptrtree[l].qend(); it++)
        {
            if(n1->upper != nullptr && n1->lower != nullptr)
                break;

            n2 = rtree_t.get_node(it->second);
            x2 = (n2->x1 + n2->x2)/2;
            y2 = (n2->y1 + n2->y2)/2;
            
            offset2 = n1->vertical ? x2 : y2;

            if(n1->id == n2->id)
                continue;

            if(offset1 == offset2)
            {
                continue;
            }
            else if(offset1 < offset2)
            {
                if(n1->upper == nullptr)
                    n1->upper = n2;
            }
            else if(offset1 > offset2)
            {
                if(n1->lower == nullptr)
                    n1->lower = n2;
            }
        }
        */
    }
    
    rtree_o.pins = vector<BoxRtree>(numLayers);
    rtree_o.wires = vector<BoxRtree>(numLayers);
    rtree_o.obstacles = vector<BoxRtree>(numLayers);

    rtree_o.db[0] = ckt->originX;
    rtree_o.db[1] = ckt->originY;
    rtree_o.db[2] = ckt->originX + ckt->width;
    rtree_o.db[3] = ckt->originY + ckt->height;

    // for pins
    #pragma omp parallel for num_threads(NUM_THREADS)
    for(int i=0; i < ckt->pins.size(); i++)
    {
        Pin* curPin = &ckt->pins[i];
        box pinShape(pt(curPin->llx, curPin->lly), pt(curPin->urx, curPin->ury));
        int bitid = rou->pin2bit[i];
        #pragma omp critical(PIN)
        {
            rtree_o.pins[curPin->l].insert( {pinShape, bitid} );
        }
    }

    // for obstacles
    #pragma omp parallel for num_threads(NUM_THREADS)
    for(int i=0; i < ckt->obstacles.size(); i++)
    {
        Obstacle* curObs = &ckt->obstacles[i];
        box obsShape(pt(curObs->llx, curObs->lly), pt(curObs->urx, curObs->ury));
        #pragma omp critical(OBS)
        {
            rtree_o.obstacles[curObs->l].insert( {obsShape, OBSTACLE} );
        }
    }

    // for wires

    //
    rtree_o.empties = vector<SegRtree>(numLayers);
    #pragma omp parallel for num_threads(NUM_THREADS)
    for(int i=0; i < ckt->empties.size(); i++)
    {
        Empty* curEmp = &ckt->empties[i];
        seg empSeg(pt(curEmp->llx, curEmp->lly), pt(curEmp->urx, curEmp->ury));
        #pragma omp critical(EMPTY)
        {
            rtree_o.empties[curEmp->l].insert( {empSeg, 0} );
        }
    } 

#ifdef DEBUG_INIT
    int leafCount = 0;
    for(int i=0; i < numTracks; i++)
    {
        cout << endl;
        //printf("\n");
        RtreeNode* node = rtree_t.get_node(i);
        if(node->lower == nullptr || node->upper == nullptr)
        {
            printf("(1) Node %d (%d %d) (%d %d) M%d #neighbors %d [LEAF]\n", i, node->x1, node->y1, node->x2, node->y2, node->l, node->neighbor.size());
            leafCount++;
        }else 
            printf("(1) Node %d (%d %d) (%d %d) M%d #neighbors %d\n", i, node->x1, node->y1, node->x2, node->y2, node->l, node->neighbor.size());
        if(node->lower != nullptr)
        {
            printf("(L) Node %d (%d %d) (%d %d) M%d\n", node->lower->id, node->lower->x1, node->lower->y1, node->lower->x2, node->lower->y2, node->lower->l);
        }

        if(node->upper != nullptr)
        {
            printf("(U) Node %d (%d %d) (%d %d) M%d\n", node->upper->id, node->upper->x1, node->upper->y1, node->upper->x2, node->upper->y2, node->upper->l);
        }

        if(node->prev != nullptr)
        {
            printf("(P) Node %d (%d %d) (%d %d) M%d\n", node->prev->id, node->prev->x1, node->prev->y1, node->prev->x2, node->prev->y2, node->prev->l);

        }

        if(node->next != nullptr)
        {
            printf("(N) Node %d (%d %d) (%d %d) M%d\n", node->next->id, node->next->x1, node->next->y1, node->next->x2, node->next->y2, node->next->l);

        }
        cout << endl;
    }
    printf("[INFO] LEAF COUNT %d\n", leafCount);
    //debug_rtree();
#endif

}



void OABusRouter::Router::construct_3d_tile_grids()
{





}



void OABusRouter::Router::debug_rtree()
{
    cout << "rtree debug start" << endl;

    for(int i=0; i < rtree_t.nodes.size(); i++)
    {
        RtreeNode* n1 = rtree_t.get_node(i);
        for(int j=0; j < n1->neighbor.size(); j++)
        {
            RtreeNode* n2 = rtree_t.get_node(n1->neighbor[j]);
            
            seg s1(pt(n1->x1, n1->y1), pt(n1->x2, n1->y2));
            seg s2(pt(n2->x1, n2->y1), pt(n2->x2, n2->y2));

            if(!bg::intersects(s1, s2))
            {
                cout << "invalid neighbor..." << endl;
                exit(0);
            }

            int x = n1->intersection[n2->id].first;
            int y = n1->intersection[n2->id].second;

            if(!bg::intersects(s1, pt(x,y)))
            {
                cout << "invalid intersection 1" << endl;
                exit(0);
            }

            if(!bg::intersects(s2, pt(x,y)))
            {
                cout << "invalid intersection 2" << endl;
                exit(0);
            }

            x = n2->intersection[n1->id].first;
            y = n2->intersection[n1->id].second;

            if(!bg::intersects(s1, pt(x,y)))
            {
                cout << "invalid intersection 1" << endl;
                exit(0);
            }

            if(!bg::intersects(s2, pt(x,y)))
            {
                cout << "invalid intersection 2" << endl;
                exit(0);
            }


        }
    }


    /*
    for(auto it : rtree_t.edges)
    {
        int n1 = it.first;
        int n2 = it.second;

        printf("e (%d %d)\n", n1, n2); 
    }
    */
    cout << "debug done" << endl;

}


        /*
         *
         *
    #pragma omp parallel for num_threads(NUM_THREADS)
    for(int i=0; i < numTracks; i++)
    {
        Track* curtrack = &ckt->tracks[i];
        RtreeNode* n1 = rtree_t.get_node(i);
        n1->id = i;
        n1->offset = curtrack->offset;
        n1->x1 = curtrack->llx;
        n1->y1 = curtrack->lly;
        n1->x2 = curtrack->urx;
        n1->y2 = curtrack->ury;
        n1->l = curtrack->l;
        n1->vertical = is_vertical(curtrack->l);
        n1->width = curtrack->width;
        n1->constraint = curtrack->constraint;
        
        seg s1(pt(n1->x1, n1->y1), pt(n1->x2, n1->y2));
        pt p1((n1->x1 + n1->x2)/2, (n1->y1 + n1->y2)/2);

        #pragma omp critical(GLOBAL)
        {
            //printf("n%d (%d %d) (%d %d) M%d\n", n1->id, n1->x1, n1->y1, n1->x2, n1->y2, n1->l);
            //cout << bg::dsv(s1) << endl;
            //cout << (int)bg::get<0,0>(s1) << " " << (int)bg::get<0,1>(s1) << " " << (int)bg::get<1,0>(s1) << " " << (int)bg::get<1,1>(s1) << endl;
            //cout << endl;
            rtree_t[n1->l]->insert({s1, n1->id});
            ptrtree[n1->l].insert({p1, n1->id});
        }
    }


            dense_hash_map<int, bi::interval_set<int>> width2interval;
            width2interval.set_empty_key(INT_MAX);

            for(auto& trackid : offset2tracks[offset])
            {
                Track* curT = &tracks[trackid];
                
                if(curLayer->direction == VERTICAL)
                    width2interval[curT->width] += bi::interval<int>::closed(curT->lly, curT->ury);
                else
                    width2interval[curT->width] += bi::interval<int>::closed(curT->llx, curT->urx);
            }

            for(int k=0; k < types.size(); k++)
            {
                bi::interval_set<int> curSet = width2interval[types[k]];

                bi::interval_set<int>::iterator it = curSet.begin();
                while(it != curSet.end())
                {
                    Track newTrack;
                    //newTrack.id = tmp.size();
                    newTrack.width = types[k];
                    newTrack.offset = offsets[j];
                    newTrack.llx = is_vertical(curLayer->id) ? offset : it->lower();
                    newTrack.urx = is_vertical(curLayer->id) ? offset : it->upper();
                    newTrack.lly = is_vertical(curLayer->id) ? it->lower() : offset;
                    newTrack.ury = is_vertical(curLayer->id) ? it->upper() : offset;
                    newTrack.l = curLayer->id; 
                    #pragma omp critical(GLOBAL)
                    {
                        newTrack.id = newTracks.size();
                        curLayer->tracks.push_back(newTrack.id);
                        newTracks.push_back(newTrack);
                    }
                    it++;
                }

                
                for(int l=k+1; l < offsets.size(); l++)
                {
                    width2interval[types[l]] -= curSet;
                }
                
            }
        }
        */

 

        /*
        #pragma omp critical(GLOBAL)
        {
            for(int j=0; j < tmp.size(); j++)
            {
                tmp[j].id = newTracks.size(); 
                curLayer->tracks.push_back(tmp[j].id);
                newTracks.push_back(tmp[j]);
            }
        }
        

        sort(tmp.begin(), tmp.end(), [&, this, curLayer](int left, int right){
                Track* t1 = &this->tracks[left];
                Track* t2 = &this->tracks[right];
                int length1 = manhatan_distance(t1->llx, t1->lly, t1->urx, t1->ury);
                int length2 = manhatan_distance(t2->llx, t2->lly, t2->urx, t2->ury);
                if(curLayer->direction == VERTICAL)
                    return (t1->lly < t2->lly) || ((t1->lly == t2->lly) && (length1 >= length2));
                else
                    return (t1->llx < t2->llx) || ((t1->llx == t2->llx) && (length1 >= length2));
                });


        for(int a=0; a < tmp.size(); a++)
        {
            Track* t1 = &tracks[tmp[a]];

            if(done.find(t1->id) != done.end())
                continue;

            
            Track newTrack(*t1);
            done.insert(t1->id);

            if(a != tmp.size()-1)
            {
                for(int b=a+1; b < tmp.size(); b++)
                {
                    Track* t2 = &tracks[tmp[b]];
                    
                    if(done.find(t2->id) != done.end())
                        continue;

                    if(curLayer->direction == VERTICAL)
                    {
                        if(t1->lly != t2->lly)
                            break;
                    }
                    else
                    {
                        if(t1->llx != t2->llx)
                            break;
                    }

                    if(t1->offset != t2->offset)
                        continue;

                    if(curLayer->direction == VERTICAL)
                    {
                        if(t1->ury >= t2->ury)
                        {
                            newTrack.width = max(newTrack.width, t2->width);
                            done.insert(t2->id);
                        }
                    }
                    else
                    {
                        if(t1->urx >= t2->urx)
                        {
                            newTrack.width = max(newTrack.width, t2->width);
                            done.insert(t2->id);
                        }
                    }
                }
            }

            #pragma omp critical(GLOBAL)
            {
                newTrack.id = newTracks.size();
                newTracks.push_back(newTrack);
                curLayer->tracks.push_back(newTrack.id);
            }
        }
        */
