#include "circuit.h"
#include "route.h"

#define DEBUG
#define BLOCK_EN

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
    dot_def << "VERSION 5.8 ;" << endl;
    dot_def << "DIVIDERCHAR \"/\" ;" << endl;
    dot_def << "BUSBITCHARS \"[]\" ;" << endl;
    dot_def << "DESIGN " << design << " ;" << endl;
    dot_def << "UNITS DISTANCE MICRONS 1000 ;" << endl;
    dot_def << endl;
    dot_def << "DIEAREA ( " << originX << " " << originY << " ) ( " << originX + width << " " << originY + height << " ) ;" << endl;
    dot_def << endl;
   
    // COMPONENTS
    dot_def << "COMPONENTS 0 ;" << endl;
    dot_def << "END COMPONENTS" << endl;
    dot_def << endl;
  
  
    for(int i=0; i < bits.size(); i++) {
        Bit* theBit = &bits[i];
        pins[theBit->pins[0]].direction = "INPUT";
    }
 
 
    //Bus* theBus = &buses[1];
  
   
    // PINS 
    dot_def << "PINS " << pins.size() << " ;" << endl;
    for(int i=0; i < pins.size(); i++) {
        Pin* thePin = &pins[i];
        //if( bits[bitHashMap[thePin->bitName]].busName != theBus->name )
        //    continue;
        int x_orig = ( thePin->llx + thePin->urx )/2;
        int y_orig = ( thePin->lly + thePin->ury )/2;
        //dot_def << "- pin_" << thePin->id << " + NET " << thePin->bitName << " + DIRECTION " << thePin->direction << " + USE SIGNAL" << endl;
        dot_def << "- pin_" << thePin->id << " + NET " << thePin->bitName << endl;
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

        //if( theBit->busName != theBus->name )
        //    continue;

        dot_def << "- " << theBit->name << endl;
        dot_def << " ";
        for(int j=0; j < theBit->pins.size(); j++) {
            Pin* thePin = &pins[theBit->pins[j]];
            dot_def << " ( PIN pin_" << thePin->id << " )";
        } dot_def << endl;

        for(int j=0; j < theBit->wires.size(); j++) {
            Wire* theWire = &rou->wires[theBit->wires[j]];
            if( j == 0 )
                dot_def << "  + ROUTED ";
            else
                dot_def << "    NEW ";
            dot_def << layers[theWire->l].name << " ( " << (theWire->x1 + theWire->x2)/2 << " " << (theWire->y1 + theWire->y2)/2 << " ) ";

            if( theWire->vertical ) {
                int length = theWire->y2 - theWire->y1;
                dot_def << "RECT ( " << 0 - theWire->width/2 << " " << 0 - length/2 << " ";
                dot_def << 0 + theWire->width/2 << " " << 0 + length/2 << " )" << endl;
            }
            else {
                int length = theWire->x2 - theWire->x1;
                dot_def << "RECT ( " << 0 - length/2 << " " << 0 - theWire->width/2 << " ";
                dot_def << 0 + length/2 << " " << 0 + theWire->width/2 << " )" << endl;
            }
            //dot_def << layers[theWire->l].name << " ( " << theWire->x1 << " " << theWire->y1 << " ) ( ";
            //dot_def  << theWire->x2 << " " << theWire->y2 << " )" << endl;
        }
         dot_def << ";" << endl;
        
    }
    dot_def << "END NETS" << endl;
    // END NETS
    dot_def << endl;


    #ifdef BLOCK_EN
    // BLOCKAGES
    dot_def << "BLOCKAGES " << obstacles.size() << " ;" << endl;
    for(int i=0; i < obstacles.size(); i++) {
        Obstacle* theBlock = &obstacles[i];
        dot_def << "    - LAYER " << layers[theBlock->l].name << endl;
        dot_def << "         RECT ( " << theBlock->llx << " " << theBlock->lly << " ) ( " << theBlock->urx << " " << theBlock->ury << " ) ;" << endl;
    }
    dot_def << "END BLOCKAGES" << endl;
    // END BLOCKAGES
    dot_def << endl;
    #endif
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

    // INITIAL LEF description
    dot_lef << "VERSION 5.8 ;" << endl;
    dot_lef << "DIVIDERCHAR \"/\" ;" << endl;
    dot_lef << "BUSBITCHARS \"[]\" ;" << endl;
    dot_lef << "UNITS" << endl;
    dot_lef << "  DATABASE MICRONS 1000 ;" << endl;
    dot_lef << "END UNITS" << endl;
    dot_lef << "MANUFACTURINGGRID 0.001 ;" << endl;
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
        dot_lef << "  PITCH " << (float)(theLayer->min_width)/500 << " ;" << endl;
        dot_lef << "  WIDTH " << (float)(theLayer->min_width)/1000 << " ;" << endl;
        dot_lef << "  SPACING " << ((float)theLayer->spacing)/1000 << " ;" << endl;
        dot_lef << "END " << theLayer->name << endl;
        dot_lef << endl;

        dot_lef << "LAYER " << theLayer->name << "_cut" << endl;
        dot_lef << "  TYPE CUT ;" << endl;
        dot_lef << "  SPACING " << ((float)theLayer->spacing)/1000 << " ;" << endl;
        dot_lef << "  WIDTH " << (float)(theLayer->min_width)/1000 << " ;" << endl;
        dot_lef << "END " << theLayer->name << "_cut" << endl;
        dot_lef << endl;

    }

    return;
}

void Circuit::out_write(){ 

    return;
}

