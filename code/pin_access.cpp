#include "circuit.h"
#include "route.h"

#define DEBUG

using namespace std;
using namespace OABusRouter;

void Circuit::pin_access() {
    for(int i=0; i < buses.size(); i++)
        pin_access(buses[i].name);
    return;
}

void Circuit::pin_access(string busName) {



    return;
}

void Circuit::debug() {

    for(int i=0; i < multipins.size(); i++){
        MultiPin* mp = &multipins[i];
        if( mp->busid == 1 ) {
            Segment* Seg = &rou->segs[rou->multipin2seg[mp->id]];

            cout << " seg layer : " << layers[Seg->l].name << endl;
            cout << " mp layer : " << layers[mp->l].name << endl;
            cout << " seg direction : " << layers[Seg->l].direction << endl;
            cout << " mp align : " << mp->align << endl;
            cout << " needVia : " << mp->needVia << endl;
            cout << " - - - - - - - - - - " << endl;
        }
    }

    return;
}
