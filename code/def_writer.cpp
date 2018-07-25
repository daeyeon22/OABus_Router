#include "circuit.h"
#include "route.h"

#define DEBUG

using namespace std;
using namespace OABusRouter;

void Circuit::def_write() {
    return def_write("output.def");
}

void Circuit::def_write(string file_name) {

    ofstream dot_def(file_name.c_str());

    if(!dot_def.good()) {
        cerr << "def_writer :: cannot open '" << file_name << "' for writing. " << endl;
        #ifdef DEBUG
            exit(0);
        #endif
    }

    // INITIAL DEF description
    dot_def << "VERSION 5.7 ;" << endl;
    dot_def << "DIVIDERCHAR \"/\" ;" << endl;
    dot_def << "BUSBITCHARS \"[]\" ;" << endl;
    dot_def << "DESIGN " << design << " ;" << endl;
    dot_def << "UNITS DISTANCE MICRONS 1000 ;" << endl;
    dot_def << endl;
    dot_def << "DIEAREA ( " << originX << " " << originY << " ) ( " << originX + width << " " << originY + height << " ) ;" << endl;
    dot_def << endl;
   
   
    // PINS 
    dot_def << "PINS " << pins.size() << " ;" << endl;
    for(int i=0; i < pins.size(); i++) {
        Pin* thePin = &pins[i];
        int x_orig = ( thePin->llx + thePin->urx )/2;
        int y_orig = ( thePin->lly + thePin->ury )/2;
        dot_def << "- pin_" << thePin->id << " + " << "net_" << thePin->bitName << endl;
        dot_def << "  + LAYER " << layers[thePin->l].name << " ( " << thePin->llx - x_orig << " " << thePin->lly - y_orig << " )";
        dot_def << " ( " << thePin->urx - x_orig << " " << thePin->ury - y_orig << " )" << endl;
        dot_def << "  + PLACED ( " << x_orig << " " << y_orig << " ) N ;" << endl;
    }
    dot_def << "END PINS" << endl;
    // END PINS
    
    dot_def << endl;


    for(int i=0; i < rou->wires.size(); i++) {
        Wire* theWire = &rou->wires[i];
        Bit* theBit = &bits[theWire->bitid];
        theBit->wires.push_back(theWire->id);
    }

    // NETS
    dot_def << "NETS " << bits.size() << " ;" << endl;
    for(int i=0; i < bits.size(); i++) {
        Bit* theBit = &bits[i];
        dot_def << "- net_" << theBit->name << endl;
        dot_def << " ";
        for(int j=0; j < theBit->pins.size(); j++) {
            Pin* thePin = &pins[theBit->pins[j]];
            dot_def << " ( PIN pin_" << thePin->id << " )";
        } dot_def << endl;
        for(int j=0; j < theBit->wires.size(); j++) {
            Wire* theWire = &rou->wires[theBit->wires[j]];
            if( j == 0 )
                dot_def << "  + ROUTED RECT ";
            else
                dot_def << "    NEW RECT ";

            if( theWire->vertical ) {
                dot_def << "( " << theWire->x1 - theWire->width/2 << " " << theWire->y1 << " ) ";
                dot_def << "( " << theWire->x2 + theWire->width/2 << " " << theWire->y2 << " )" << endl;
            }
            else {
                dot_def << "( " << theWire->x1 << " " << theWire->y1 - theWire->width/2 << " ) ";
                dot_def << "( " << theWire->x2 << " " << theWire->y2 + theWire->width/2 << " )" << endl;
            }
            //dot_def << layers[theWire->l].name << " ( " << theWire->x1 << " " << theWire->y1 << " ) ( ";
            //dot_def  << theWire->x2 << " " << theWire->y2 << " )" << endl;
        } dot_def << ";" << endl;
    }
    dot_def << "END NETS" << endl;
    // END NETS
    dot_def << endl;
    dot_def << "END DESIGN" << endl;
    // END DESIGN

    return;
}

void Circuit::lef_write() {
    return lef_write("output.lef");
}

void Circuit::lef_write(string file_name) {

    ofstream dot_lef(file_name.c_str());

    if(!dot_lef.good()) {
        cerr << "lef_writer :: cannot open '" << file_name << "' for writing. " << endl;
        #ifdef DEBUG
            exit(0);
        #endif
    }

    // INITIAL DEF description
    dot_lef << "VERSION 5.7 ;" << endl;
    dot_lef << "DIVIDERCHAR \"/\" ;" << endl;
    dot_lef << "BUSBITCHARS \"[]\" ;" << endl;
    dot_lef << "UNITS" << endl;
    dot_lef << "  DATABASE MICRONS 1000 ;" << endl;
    dot_lef << "END UNITS" << endl;
    dot_lef << endl;
  
    for(int i=0; i < layers.size(); i++) {
        Layer* theLayer = &layers[i];
        dot_lef << "LAYER " << theLayer->name << endl;
        dot_lef << "  TYPE ROUTING ;" << endl;
        dot_lef << "  DIRECTION ";
        if( theLayer->is_vertical() )
            dot_lef << "VERTICAL ;" << endl;
        else
            dot_lef << "HORIZONTAL ;" << endl;
        dot_lef << "  MINWIDTH " << theLayer->track_min_width/1000 << " ;" << endl;
        dot_lef << "  SPACING " << theLayer->spacing/1000 << " ;" << endl;
        dot_lef << "END " << theLayer->name << endl;
        dot_lef << endl;
    }

    return;
}

