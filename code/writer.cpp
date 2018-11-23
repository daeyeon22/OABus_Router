#include "circuit.h"
#include "route.h"
#include "func.h"


#define DEBUG
#define BLOCK_EN

using namespace std;
using namespace OABusRouter;

using OABusRouter::inverse_vector;
//void Circuit::def_write() {
//    return def_write("../output/output.def");
//}


void Circuit::def_write(string dirPos, string benchName)
{

    string file_name = dirPos + "/" + benchName + ".def";

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

    //for(int i=0; i < rou->wires.size(); i++) {
    //    Wire* theWire = &rou->wires[i];
    //    Bit* theBit = &bits[theWire->bitid];
    //    theBit->wires.push_back(theWire->id);
    //}
    /////////////////////////////////////////////////////
    //for(int i=0; i < rou->vias.size(); i++)
    //{
    //    Via* theVia = &rou->vias[i];
    //    Bit* theBit = &bits[rou->wires[theVia->w1].bitid];
    //    theBit->vias.push_back(theVia->id);
    //}
    /////////////////////////////////////////////////////

    // NETS
    dot_def << "NETS " << bits.size() << " ;" << endl;
    for(int i=0; i < bits.size(); i++) {
        Bit* theBit = &bits[i];
        Bus* theBus = &buses[busHashMap[theBit->busName]];

        //if( theBit->busName != theBus->name )
        //    continue;

        dot_def << "- " << theBit->name << endl;
        dot_def << " ";
        for(int j=0; j < theBit->pins.size(); j++) {
            Pin* thePin = &pins[theBit->pins[j]];
            dot_def << " ( PIN pin_" << thePin->id << " )";
        } dot_def << endl;

        for(int j=0; j < theBit->paths.size(); j++)
        {
            Path* thePath = &theBit->paths[j];
            if( j == 0 )
                dot_def << "  + ROUTED ";
            
            if(thePath->via)
            {

            }
            else
            {
                if( j != 0 )
                    dot_def << "    NEW ";

                dot_def << layers[thePath->l].name << " ( " << (thePath->x[0] + thePath->x[1])/2 << " " << (thePath->y[0] + thePath->y[1])/2 << " ) ";
                int width = theBus->width[thePath->l];

                if(is_vertical(thePath->l))
                {
                    int length = thePath->y[1] - thePath->y[0];
                    dot_def << "RECT ( " << 0 - width/2 << " " << 0 - length/2 << " ";
                    dot_def << 0 + width/2 << " " << 0 + length/2 << " )" << endl;
                }
                else {
                    int length = thePath->x[1] - thePath->x[0];
                    dot_def << "RECT ( " << 0 - length/2 << " " << 0 - width/2 << " ";
                    dot_def << 0 + length/2 << " " << 0 + width/2 << " )" << endl;
                }
            }
            

        }

        /*
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
        */
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

    /////////////////////////////////////
    for(int i=0; i < buses.size(); i++)
    {
        if(buses[i].assign)
            def_write(i, dirPos, benchName);
    }
    ////////////////////////////////////
    return;
}

void Circuit::def_write(int busid, string dirPos, string benchName)
{

    Bus* curBus = &buses[busid];
    string file_name = dirPos + "/" + benchName + "/" + curBus->name + ".def";

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
  
  
    
    for(int i=0; i < curBus->bits.size(); i++) {
        Bit* theBit = &bits[curBus->bits[i]];
        pins[theBit->pins[0]].direction = "INPUT";
    }
 
 
    //Bus* theBus = &buses[1];
  
   
    // PINS 
    dot_def << "PINS " << curBus->numBits * curBus->numPinShapes << " ;" << endl;
    for(int i=0; i < curBus->numBits;  i++) {
        Bit* curBit = &bits[curBus->bits[i]];
        for(int j=0; j < curBit->pins.size(); j++)
        {
            Pin* thePin = &pins[curBit->pins[j]];
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
    }
    dot_def << "END PINS" << endl;
    // END PINS
    dot_def << endl;


    /////////////////////////////////////////////////////
    //for(int i=0; i < rou->vias.size(); i++)
    //{
    //    Via* theVia = &rou->vias[i];
    //    Bit* theBit = &bits[rou->wires[theVia->w1].bitid];
    //    theBit->vias.push_back(theVia->id);
    //}
    /////////////////////////////////////////////////////

    // NETS
    dot_def << "NETS " << curBus->bits.size() << " ;" << endl;
    for(int i=0; i < curBus->bits.size(); i++) {
        Bit* theBit = &bits[curBus->bits[i]];
        Bus* theBus = curBus;
        //&buses[busHashMap[theBit->busName]];

        //if( theBit->busName != theBus->name )
        //    continue;

        dot_def << "- " << theBit->name << endl;
        dot_def << " ";
        for(int j=0; j < theBit->pins.size(); j++) {
            Pin* thePin = &pins[theBit->pins[j]];
            dot_def << " ( PIN pin_" << thePin->id << " )";
        } dot_def << endl;

        for(int j=0; j < theBit->paths.size(); j++)
        {
            Path* thePath = &theBit->paths[j];
            if( j == 0 )
                dot_def << "  + ROUTED ";
            
            if(thePath->via)
            {

            }
            else
            {
                if( j != 0 )
                    dot_def << "    NEW ";

                dot_def << layers[thePath->l].name << " ( " << (thePath->x[0] + thePath->x[1])/2 << " " << (thePath->y[0] + thePath->y[1])/2 << " ) ";
                int width = theBus->width[thePath->l];

                if(is_vertical(thePath->l))
                {
                    int length = thePath->y[1] - thePath->y[0];
                    dot_def << "RECT ( " << 0 - width/2 << " " << 0 - length/2 << " ";
                    dot_def << 0 + width/2 << " " << 0 + length/2 << " )" << endl;
                }
                else {
                    int length = thePath->x[1] - thePath->x[0];
                    dot_def << "RECT ( " << 0 - length/2 << " " << 0 - width/2 << " ";
                    dot_def << 0 + length/2 << " " << 0 + width/2 << " )" << endl;
                }
            }
            

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

//void Circuit::lef_write() {
//    return lef_write("../output/output.lef");
//}


void Circuit::lef_write(string dirPos, string benchName)
{
    string file_name = dirPos + "/" + benchName + ".lef";
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

    /////////////////
    for(int i=0; i < buses.size(); i++)
    {
        if(buses[i].assign)
            lef_write(i, dirPos, benchName);
    }

    return;
}

void Circuit::lef_write(int busid, string dirPos, string benchName) 
{
    string file_name = dirPos + "/" + benchName + "/" + buses[busid].name + ".lef";
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

void Circuit::out_write(string file_name){ 

    FILE* dot_out = fopen(file_name.c_str(), "w");
    //ofstream dot_out(file_name.c_str());

    int visit[rou->wires.size()];
    memset(visit, -1, rou->wires.size());

    for(int i=0; i < buses.size(); i++)
    {
        Bus* curB = &buses[i];
        fprintf(dot_out, "BUS %s\n", curB->name.c_str());

        for(int j =0; j < curB->numBits; j++)
        {
            Bit* curBit = &bits[curB->bits[j]];
            fprintf(dot_out, "BIT %s\n", curBit->name.c_str());
            fprintf(dot_out, "PATH %d\n", curBit->paths.size());
            for(auto& p : curBit->paths)
            {
                if(p.via)
                    fprintf(dot_out, "%s (%d %d)\n", layers[p.l].name.c_str(), p.x[0], p.y[0]);
                else
                    fprintf(dot_out, "%s (%d %d) (%d %d)\n", layers[p.l].name.c_str(), p.x[0], p.y[0], p.x[1], p.y[1]);
            }
            fprintf(dot_out, "ENDPATH\n");
            fprintf(dot_out, "ENDBIT\n");

        }
        fprintf(dot_out, "ENDBUS\n");
    }

    fclose(dot_out);

    return;
}


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

        Bus* curBus = &buses[i];
        if(curBus->flip)
        {
            inverse_vector(curBus->bits);
            curBus->flip = false;
        }

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
                Router* router = rou;
                auto cmp = [&,router,wireid,this](const pair<int, pair<int,int>> &left, const pair<int, pair<int,int>> &right)
                {
                    bool equal = (left.second.first == right.second.first) && (left.second.second == right.second.second);
                    if(equal)
                    {
                        int id1 = left.first;
                        int id2 = right.first;
                        int l1, l2;
                        l1 = (id1 != PINTYPE) ? router->wires[id1].l : this->pins[router->wire2pin[wireid]].l;
                        l2 = (id2 != PINTYPE) ? router->wires[id2].l : this->pins[router->wire2pin[wireid]].l;
                        return l1 < l2;
                    }
                    else
                    {
                        return (left.second.first < right.second.first || left.second.second < right.second.second);
                    }
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
#ifdef DEBUG_PATH
                printf("%s wire%d -> #intersection %d\n",
                        curpin->bitName.c_str(), w1->id, intersection.size());

                if(w1->intersection.size() != intersection.size())
                {
                    cout << w1->intersection.size() << endl;
                    cout << intersection.size() << endl;
                    exit(0);
                }
#endif

                for(auto& it : intersection)
                {
                    x = it.second.first;
                    y = it.second.second;
                    
                    if(it.first == PINTYPE)
                    {
                        pinid = rou->wire2pin[w1->id];
#ifdef DEBUG_PATH
                        printf("%s wire%d (%d %d) (%d %d) M%d connected with pin%d at (%d %d) M%d\n",
                                curpin->bitName.c_str(),
                                w1->id, w1->x1, w1->y1, w1->x2, w1->y2, w1->l,
                                pinid, x, y, pins[pinid].l);
#endif

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
#ifdef DEBUG_PATH
                            printf("%s wire%d (%d %d) (%d %d) M%d connected with wire%d at (%d %d) M%d\n",
                                curpin->bitName.c_str(),
                                w1->id, w1->x1, w1->y1, w1->x2, w1->y2, w1->l,
                                w2->id, x, y, w2->l);
#endif

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
                        else
                        {
#ifdef DEBUG_PATH
                           printf("%s wire%d already visited!\n",
                                   curpin->bitName.c_str(), w2->id);
#endif
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


#ifdef DEBUG_PATH
    for(int k=0; k < rou->wires.size(); k++)
    {
        if(!visit[k])
        {
            printf("wire %d unvisit\n", k);
            exit(0);
        }
    }
#endif


}





