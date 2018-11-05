
#include "tree.h"
#include "typedef.h"
#include "circuit.h"
#include "route.h"
#include "func.h"


//#define DEBUG_RTREE_T
using OABusRouter::design_ruled_area;
using OABusRouter::is_vertical;



bool OABusRouter::Rtree_o::insert(int type, int bitid, int x1, int y1, int x2, int y2, int l)
{
    box elem(pt(x1, y1), pt(x2, y2));
    switch(type)
    {
        case OBSTACLE:
            obstacles[l].insert( {elem, bitid} );
            break;

        case PINTYPE:
            pins[l].insert( {elem, bitid} );
            break;
            
        case WIRETYPE:
            wires[l].insert( {elem, bitid} );
            break;
    
        default:
            return false;
            break;
    }
    
    return true;
}

bool OABusRouter::Rtree_o::remove(int type, int bitid, int x1, int y1, int x2, int y2, int l)
{
    box elem(pt(x1, y1), pt(x2, y2));
    switch(type)
    {
        case OBSTACLE:
            obstacles[l].remove( {elem, bitid} );
            break;

        case PINTYPE:
            pins[l].remove( {elem, bitid} );
            break;
            
        case WIRETYPE:
            wires[l].remove( {elem, bitid} );
            break;
    
        default:
            return false;
            break;
    }
    
    return true;
}

bool OABusRouter::Rtree_o::insert(int type, int bitid, int x1, int y1, int x2, int y2, int l, int width)
{
    int xDrd[2] = { min(x1, x2), max(x1, x2) };
    int yDrd[2] = { min(y1, y2), max(y1, y2) };
    design_ruled_area(xDrd, yDrd, width, 0, is_vertical(l));
    return insert(type, bitid, xDrd, yDrd, l);
}

bool OABusRouter::Rtree_o::remove(int type, int bitid, int x1, int y1, int x2, int y2, int l, int width)
{
    int xDrd[2] = { min(x1, x2), max(x1, x2) };
    int yDrd[2] = { min(y1, y2), max(y1, y2) };
    design_ruled_area(xDrd, yDrd, width, 0, is_vertical(l));
    return remove(type, bitid, xDrd, yDrd, l);
}


bool OABusRouter::Rtree_o::insert(int type, int bitid, int x[], int y[], int l)
{
    return insert(type, bitid, x[0], y[0], x[1], y[1], l);    
}

bool OABusRouter::Rtree_o::remove(int type, int bitid, int x[], int y[], int l)
{
    return remove(type, bitid, x[0], y[0], x[1], y[1], l);
}


bool OABusRouter::Rtree_o::spacing_violation_clean(int bitid, int x1, int y1, int x2, int y2, int l, int width, int spacing, bool vertical)
{
    return (num_spacing_violation(bitid, x1, y1, x2, y2, l, width, spacing, vertical) == 0) ? true : false;
}


int OABusRouter::Rtree_o::num_spacing_violation(int bitid, int x1, int y1, int x2, int y2, int l, int width, int spacing, bool vertical)
{
    int x[2] = { min(x1, x2), max(x1, x2) };
    int y[2] = { min(y1, y2), max(y1, y2) };
    return num_spacing_violation(bitid, x, y, l, width, spacing, vertical);
}


int OABusRouter::Rtree_o::num_spacing_violation(int bitid, int x[], int y[], int l, int width, int spacing, bool vertical)
{
    int totalSPV = 0;

    int xExt1[2], yExt1[2];
    int xExt2[2], yExt2[2];
    xExt1[0] = (vertical) ? x[0] - (width/2 + spacing) : x[0] - spacing;
    xExt1[1] = (vertical) ? x[1] + (width/2 + spacing) : x[1] + spacing;
    yExt1[0] = (vertical) ? y[0] - spacing : y[0] - (width/2 + spacing);
    yExt1[1] = (vertical) ? y[1] + spacing : y[1] + (width/2 + spacing);
    xExt2[0] = (vertical) ? x[0] - (width/2) : x[0];
    xExt2[1] = (vertical) ? x[1] + (width/2) : x[1];
    yExt2[0] = (vertical) ? y[0] : y[0] - (width/2);
    yExt2[1] = (vertical) ? y[1] : y[1] + (width/2);
    box wire_area_with_spac(pt(xExt1[0], yExt1[0]), pt(xExt1[1], yExt1[1]));
    box wire_area_no_spac(pt(xExt2[0], yExt2[0]), pt(xExt2[1], yExt2[1]));

    // DESIGN BOUNDARY
    box die_area(pt(db[0], db[1]), pt(db[2], db[3]));
    if(!bg::within(wire_area_with_spac, die_area))
        totalSPV++;

    vector<pair<box,int>> queries;
    // PINS
    queries.clear();
    pins[l].query(bgi::intersects(wire_area_with_spac), back_inserter(queries));
    for(auto& it : queries)
    {
        if(bg::touches(it.first, wire_area_with_spac))
            continue;

        if(it.second != bitid)
            totalSPV++;
    }
    // WIRES
    queries.clear();
    wires[l].query(bgi::intersects(wire_area_with_spac), back_inserter(queries));
    for(auto& it : queries)
    {
        if(bg::touches(it.first, wire_area_with_spac))
            continue;

        if(it.second != bitid)
            totalSPV++;
        else
        {
            if(bg::overlaps(wire_area_no_spac, it.first))
                totalSPV++;
        }
    }

    // OBSTACLE
    queries.clear();
    obstacles[l].query(bgi::intersects(wire_area_with_spac), back_inserter(queries));
    totalSPV += queries.size();

    return totalSPV;
}

bool OABusRouter::Rtree_t::get_extension(int n1, int& n2, bool lCorner)
{
    return (lCorner) ? prev(n1, n2) : next(n1, n2);
}

bool OABusRouter::Rtree_t::next(int current, int& next)
{
    RtreeNode* curNode = &nodes[current];
    if(curNode->next != nullptr)
    {
        next = curNode->next->id;
        return true;
    }
    return false;
}

bool OABusRouter::Rtree_t::prev(int current, int &prev)
{
    RtreeNode* curNode = &nodes[current];
    if(curNode->prev != nullptr)
    {
        prev = curNode->prev->id;
        return true;
    }
    return false;
}


bool OABusRouter::Rtree_t::corner(int current, int &target, int l, bool lCorner)
{
    RtreeNode* curNode = get_node(current);
    
    vector<pair<seg,int>> queries;
    
    if(lCorner)
        trees[l].query(bgi::intersects(pt(curNode->x1, curNode->y1)), back_inserter(queries));
    else
        trees[l].query(bgi::intersects(pt(curNode->x2, curNode->y2)), back_inserter(queries));

    for(auto& it : queries)
    {
        target == it.second;
        if(target == current)
            continue;
        else
        {
            cout << bg::dsv(it.first) << endl;
            cout << target << endl;
            return true;
        }
    }
    return false;
}


bool OABusRouter::Rtree_t::search_node_debug(int current, int &next, int neighbor, int width, int spacing, bool upper)
{
    RtreeNode *n1, *n2;
    n1 = get_node(current);

    bool prevExt = n1->prev != nullptr ? true : false;
    bool nextExt = n1->next != nullptr ? true : false;

    int count = 0; //maxCount = 5;
    for(n2 = upper ? n1->upper : n1->lower; n2 != nullptr; n2 = upper ? n2->upper : n2->lower)
    {
        if(count++ > 5)
            return false;

        cout << "iter => " << n2->id << endl;
        
            
        bool cond1 = (abs(n1->offset - n2->offset) >= width + spacing) ? true : false;
        bool cond2 = (n2->width >= width) ? true : false;
        bool cond3 = is_valid( {neighbor, n2->id} );
        bool cond4 = (n2->prev != nullptr) == prevExt ? true : false;
        bool cond5 = (n2->next != nullptr) == nextExt ? true : false;
        if(cond1 && cond2 && cond3 && cond4 && cond5)
        {
            next = n2->id;
            return true;
        }
    }
   
    return false;
}



bool OABusRouter::Rtree_t::search_node2(int current, int &next, int neighbor, int width, int spacing, bool upper)
{

    RtreeNode *n_init = get_node(current);
 
    bool valid = false;
    bool prevExt = n_init->prev != nullptr ? true : false;
    bool nextExt = n_init->next != nullptr ? true : false;


    printf("- - - - - - - Search Node - - - - - - -\n");
    printf("n_curr : n%d (%d %d) (%d %d) M%d\n", n_init->id, n_init->x1, n_init->y1, n_init->x2, n_init->y2, n_init->l);

    if(upper)
    {
        if(n_init->upper != nullptr)
            current = n_init->upper->id;
        else
            return false;
    }
    else
    {
        if(n_init->lower != nullptr)
            current = n_init->lower->id;
        else
            return false;
    }

    for(int count = 0; count < 5; count++)
    {
        RtreeNode* n_cur = get_node(current); //upper ? n_init->upper : n_cur->lower; //n_init->next; //get_node(current);
        printf("n_next : n%d (%d %d) (%d %d) M%d\n", n_cur->id, n_cur->x1, n_cur->y1, n_cur->x2, n_cur->y2, n_cur->l);
        bool cond1 = (abs(n_init->offset - n_cur->offset) >= width + spacing) ? true : false;
        bool cond2 = (n_cur->width >= width) ? true : false;
        bool cond3 = is_valid( {neighbor, current} );
        bool cond4 = (n_cur->prev != nullptr) == prevExt ? true : false;
        bool cond5 = (n_cur->next != nullptr) == nextExt ? true : false;

        if(cond1 && cond2 && cond3 && cond4 && cond5)
        {
            next = current;
            valid = true;
            break;
        }
        else
        {
            if(!cond1)
                printf("condition1 failed\n");
            if(!cond2)
                printf("condition2 failed\n");
            if(!cond3)
                printf("condition3 failed\n");
            if(!cond4)
                printf("condition4 failed\n");
            if(!cond5)
                printf("condition5 failed\n");
        }


        RtreeNode* n_next = upper ? n_cur->upper : n_cur->lower;
        if(n_next == nullptr)
        {   
            break;
        }

        current = n_next->id;
    }
  
    return valid;
}

bool OABusRouter::Rtree_t::search_node(RtreeNode* n1, RtreeNode* n2, RtreeNode* n_iter, int width, int spacing, bool upper)
{
    bool prevExt = n_iter->prev != nullptr ? true : false;
    bool nextExt = n_iter->next != nullptr ? true : false;
    bool found = false;

#ifdef DEBUG_RTREE_T
    printf("\n");
    printf("- - - - - - Get next node - - - - - -\n");
    printf("n_prev : n%d (%d %d) (%d %d) M%d\n", n1->id, n1->x1, n1->y1, n1->x2, n1->y2, n1->l);
    printf("n_iter : n%d (%d %d) (%d %d) M%d\n", n_iter->id, n_iter->x1, n_iter->y1, n_iter->x2, n_iter->y2, n_iter->l);
#endif

    int count = 0; //maxCount = 5;
    for(n2 = upper ? n_iter->upper : n_iter->lower; n2 != nullptr; n2 = upper ? n2->upper : n2->lower)
    {
        if(count++ > 5)
            return false;
#ifdef DEBUG_RTREE_T      
        printf("n_iter : n%d (%d %d) (%d %d) M%d\n", n2->id, n2->x1, n2->y1, n2->x2, n2->y2, n2->l);
#endif
        bool cond1 = (abs(n_iter->offset - n2->offset) >= width + spacing) ? true : false;
        bool cond2 = (n2->width >= width) ? true : false;
        bool cond3 = is_valid( { n1->id, n2->id } );
        bool cond4 = (n2->prev != nullptr) == prevExt ? true : false;
        bool cond5 = (n2->next != nullptr) == nextExt ? true : false;
        if(cond1 && cond2 && cond3 && cond4 && cond5)
        {
            found = true;
            break;
        }
    }
  
    return found;
}

bool OABusRouter::Rtree_t::search_node(int current, int &next, int neighbor, int width, int spacing, bool upper)
{
    RtreeNode *n_init = get_node(current);
 
    bool valid = false;
    bool prevExt = n_init->prev != nullptr ? true : false;
    bool nextExt = n_init->next != nullptr ? true : false;

    if(upper)
    {
        if(n_init->upper != nullptr)
            current = n_init->upper->id;
        else
            return false;
    }
    else
    {
        if(n_init->lower != nullptr)
            current = n_init->lower->id;
        else
            return false;
    }


    for(int count = 0; count < 5; count++)
    {
        RtreeNode* n_cur = get_node(current);
    
        bool cond1 = (abs(n_init->offset - n_cur->offset) >= width + spacing) ? true : false;
        bool cond2 = (n_cur->width >= width) ? true : false;
        bool cond3 = is_valid( {neighbor, current} );
        bool cond4 = (n_cur->prev != nullptr) == prevExt ? true : false;
        bool cond5 = (n_cur->next != nullptr) == nextExt ? true : false;

        if(cond1 && cond2 && cond3 && cond4 && cond5)
        {
            next = current;
            valid = true;
            break;
        }


        RtreeNode* n_next = upper ? n_cur->upper : n_cur->lower;
        if(n_next == nullptr)
        {   
            break;
        }

        current = n_next->id;
    }
  
    return valid;
}


bool OABusRouter::Rtree_t::lower(RtreeNode* n1, RtreeNode* n2, int prev, int width, int spacing)
{
    for(n2 = n1->lower; n2 != nullptr; n2 = n2->lower)
    {
        bool cond1 = (abs(n1->offset - n2->offset) >= width + spacing) ? true : false;
        bool cond2 = (n2->width >= width) ? true : false;
        bool cond3 = is_valid({min(prev, n2->id), max(prev, n2->id)});
        
        if(cond1 && cond2 && cond3)
            return true;
    }

    return false;
}

bool OABusRouter::Rtree_t::upper(RtreeNode* n1, RtreeNode* n2, int prev, int width, int spacing)
{
    for(n2 = n1->upper; n2 != nullptr; n2 = n2->upper)
    {
        bool cond1 = (abs(n1->offset - n2->offset) >= width + spacing) ? true : false;
        bool cond2 = (n2->width >= width) ? true : false;
        bool cond3 = is_valid({min(prev, n2->id), max(prev, n2->id)});

        if(cond1 && cond2 && cond3)
            return true;
    }
    return false;
}

bool OABusRouter::Rtree_t::is_valid(pair<int,int> e)
{
    pair<int,int> _e = { min(e.first, e.second), max(e.first, e.second) };
    return (edges.find(_e) == edges.end()) ? false : true;
}
