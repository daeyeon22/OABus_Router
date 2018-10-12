
#include "tree.h"
#include "typedef.h"

bool OABusRouter::Rtree_o::insert(int type, int bitid, int x[], int y[], int l)
{
    
    
    box elem(pt(x[0], y[0]), pt(x[1], y[1]));
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

bool OABusRouter::Rtree_o::remove(int type, int bitid, int x[], int y[], int l)
{
    box elem(pt(x[0], y[0]), pt(x[1], y[1]));
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

bool OABusRouter::Rtree_t::lower(RtreeNode* n1, RtreeNode* n2)
{
    if(n1->lower == nullptr)
        return false;

    n2 = n1->lower;
    return true;
}

bool OABusRouter::Rtree_t::upper(RtreeNode* n1, RtreeNode* n2)
{
    if(n2->upper == nullptr)
        return false;

    n2 = n1->upper;
    return true;
}


