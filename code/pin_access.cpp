#include "circuit.h"
#include "route.h"
#include "func.h"

#define DEBUG

using namespace std;
using namespace OABusRouter;

void Circuit::pin_access() {
    wire_track_mapping();
    pin_track_mapping();
    ContactMapping();
    for(int i=0; i < buses.size(); i++)
        pin_access(i);
    return;
}

void Circuit::pin_access(int busid) {
    Bus* bus = &buses[busid];
    for(int i=0; i < bus->multipins.size(); i++) {
        MultiPin* mp = &multipins[bus->multipins[i]];
        Segment* sg = &rou->segs[rou->multipin2seg[mp->id]];
        assert ( sg->wires.size() == mp->pins.size() );

        int dist_l = mp->l - sg->l;
        if( mp->needVia == true )
            if( mp->l == 0 )
                dist_l++;
            else
                dist_l--;
        cout << " dist l : " << dist_l << endl;

    }
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

void Circuit::pin_track_mapping() {
    int count = 0;
    for(int i=0; i < multipins.size(); i++) {
        MultiPin* MP = &multipins[i];
        Segment* Seg = &rou->segs[rou->multipin2seg[i]];
        for(int j = 0; j < MP->pins.size(); j++) {
            Pin* P = &this->pins[MP->pins[j]];
            int layer_num = P->l;
            if( MP->needVia == true ) {
                if( layer_num == 0 )
                    layer_num++;
                else
                    layer_num--; 
            }
            Layer* theLayer = &layers[layer_num];
            for(int k=0; k < theLayer->tracks.size(); k++) {
                Track* TR = &tracks[theLayer->tracks[k]];
                Point _a(P->llx,P->lly);
                Point _b(P->urx,P->ury);
                Point _c, _d;
                if( theLayer->is_vertical() ){
                    _c.x = TR->llx - TR->width/2;
                    _c.y = TR->lly;
                    _d.x = TR->urx + TR->width/2;
                    _d.y = TR->ury;
                } else {
                    _c.x = TR->llx;
                    _c.y = TR->lly - TR->width/2;
                    _d.x = TR->urx;
                    _d.y = TR->ury + TR->width/2;
                }
                if( rect_intersect(_a,_b,_c,_d) == true ) {
                    P->trackid = TR->id;
                    count++;
                    break; 
                }
            }
        }
    }
    cout << " num_pin : " << pins.size();
    cout << " count : " << count << endl;
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
    Pin* P = &pins[90];
    Layer* L = &layers[P->l];
    for(int i=0; i < L->tracks.size(); i++) {
        Track* T = &tracks[L->tracks[i]];

        pair<int,int> a(P->llx,P->lly);
        pair<int,int> b(P->urx,P->ury);
        pair<int,int> c(T->llx,T->lly);
        pair<int,int> d(T->urx,T->ury);

        if( is_intersect(make_pair(a,b),make_pair(c,d)) == true ) {
            P->trackid = T->id;
            cout << "ab : " << ccw(a,b,c) * ccw(a,b,d) << endl;
            cout << "cd : " << ccw(c,d,a) * ccw(c,d,b) << endl;

            cout << " Found !!" << endl;
            cout << " a : " << a.first << " " << a.second << endl;
            cout << " b : " << b.first << " " << b.second << endl;
            cout << " c : " << c.first << " " << c.second << endl;
            cout << " d : " << d.first << " " << d.second << endl;
        }

    }


    exit(0);
    return;
}

bool Circuit::rect_intersect(Point A_ll, Point A_ur, Point B_ll, Point B_ur) {

    bool a = A_ll.x > B_ur.x;
    bool b = A_ur.x < B_ll.x;
    bool c = A_ur.y < B_ll.y;
    bool d = A_ll.y > B_ur.y;

    if( ( a || b || c || d ) == true )
        return false;
    else
        return true;
}


int Circuit::ccw(pair<int,int> a, pair<int,int> b, pair<int,int> c)
{
    double op = a.first/100.0*b.second/100.0 + b.first/100.0*c.second/100.0 + c.first/100.0*a.second/100.0;
    op -= (a.second/100.0*b.first/100.0 + b.second/100.0*c.first/100.0 + c.second/100.0*a.first/100.0);
    //cout << op << endl;
    
    if( op > 0 ) return 1;
    else if ( op == 0 ) return 0;
    else return -1;
}


