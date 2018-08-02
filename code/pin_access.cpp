#include "circuit.h"
#include "route.h"
#include "func.h"

#define DEBUG

using namespace std;
using namespace OABusRouter;

void Circuit::pin_access() {
    wire_track_mapping();
    ContactMapping();
    for(int i=0; i < buses.size(); i++)
        pin_access(buses[i].name);
    return;
}

void Circuit::pin_access(string busName) {



    return;
}

bool Circuit::is_cross(Track* tr1, Track* tr2) {
    pair<int,int> a(tr1->llx,tr1->lly);
    pair<int,int> b(tr1->urx,tr1->ury);
    pair<int,int> c(tr2->llx,tr2->lly);
    pair<int,int> d(tr2->urx,tr2->ury);

    if( ccw(a,b,c)*ccw(a,b,d) <= 0 && ccw(c,d,a)*ccw(c,d,b) <= 0 )
        return true;
    else
        return false;
}

bool Circuit::is_intersect(Point _a, Point _b, Point _c, Point _d) {
    pair<int,int> a(_a.x,_a.y);
    pair<int,int> b(_b.x,_b.y);
    pair<int,int> c(_c.x,_c.y);
    pair<int,int> d(_d.x,_d.y);
    return is_intersect(make_pair(a,b),make_pair(c,d));
}

bool Circuit::is_intersect(Track* tr1, Track* tr2) {
    pair<int,int> a(tr1->llx,tr1->lly);
    pair<int,int> b(tr1->urx,tr1->ury);
    pair<int,int> c(tr2->llx,tr2->lly);
    pair<int,int> d(tr2->urx,tr2->ury);
    return is_intersect(make_pair(a,b),make_pair(c,d));
}

bool Circuit::is_intersect(pair<pair<int,int>,pair<int,int> > line1, pair<pair<int,int>,pair<int,int> > line2) {
    pair<int,int> a = line1.first;
    pair<int,int> b = line1.second;
    pair<int,int> c = line2.first;
    pair<int,int> d = line2.second;
    int ab = ccw(a,b,c)*ccw(a,b,d);
    int cd = ccw(c,d,a)*ccw(c,d,b);
    if( ab == 0 && cd == 0 ) {
        if( a > b ) swap(a,b);
        if( c > d ) swap(c,d);
        return c <= b && a <= d;
    }
    return ab <= 0 && cd <= 0;
}

void Circuit::wire_track_mapping() {
    for(int i=0; i < rou->wires.size(); i++) {
        Wire* theWire = &rou->wires[i];
        Track* theTrack = &tracks[theWire->trackid];
        theTrack->wires.push_back(theWire->trackid);
    }
    return;
}

void Circuit::ContactMapping() {

    for(int i=1; i < layers.size(); i++) {
        Layer* DownLayer = &layers[i-1];
        Layer* UpLayer = &layers[i];
        for(int j=0; j < UpLayer->tracks.size(); j++) {
            Track* UpTrack = &tracks[UpLayer->tracks[j]];
            Contact start;
            start.id = this->contacts.size();
            start.trackid = UpTrack->id;
            start.l = UpTrack->l;
            start.p.x = UpLayer->llx;
            start.p.y = UpLayer->lly;
            this->contacts.push_back(start);
            UpTrack->contacts.push_back(start.id);
            for(int k=0; k < DownLayer->tracks.size(); k++) {
                Track* DownTrack = &tracks[DownLayer->tracks[k]];
                if( is_cross(DownTrack,UpTrack) == true ) {
                    Contact con;
                    con.trackid = UpTrack->id;
                    con.l = UpTrack->l;
                    if( UpLayer->is_vertical() == true ) {
                        con.p.x = UpLayer->llx;
                        con.p.y = DownLayer->lly;
                    } else {
                        con.p.x = DownLayer->llx;
                        con.p.y = UpLayer->lly;
                    }
                }
            }
            Contact end;
            end.id = this->contacts.size();
            end.trackid = UpTrack->id;
            end.l = UpTrack->l;
            end.p.x = UpLayer->urx;
            end.p.y = UpLayer->ury;
            if( this->contacts[UpTrack->contacts.back()].p != end.p ) {
                this->contacts.push_back(end);
                UpTrack->contacts.push_back(end.id);
            }
        }
    }

    return;
}

void Circuit::debug() {

    for(int i=0; i < multipins.size(); i++) {
        MultiPin* mp = &multipins[i];
        Segment* sg = &rou->segs[rou->multipin2seg[i]];
        cout << " layer distance : " << mp->l - sg->l << endl;
    }

    exit(0);
    return;
}

int Circuit::ccw(pair<int,int> a, pair<int,int> b, pair<int,int> c)
{
    int op = a.first*b.second + b.first*c.second + c.first*a.second;
    op -= (a.second*b.first + b.second*c.first + c.second*a.first);
    if( op > 0)
        return 1;
    else if ( op == 0 )
        return 0;
    else
        return -1;
}


