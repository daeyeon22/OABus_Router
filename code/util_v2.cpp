#include "circuit.h"
#include "util.h"
#include "func.h"
#include "route.h"

#define DEBUG


using namespace svg;
namespace br = OABusRouter;

static string plotdir = "./plot/";
static string plotall = "./plot/bus_all.svg";
static bool show_track = true;
static bool maze = false;

void OABusRouter::Router::Plot()
{

    cout << "Create plots..." << endl;
    int i, numBuses;
    numBuses = ckt->buses.size();


cout << "1" << endl;
    for(i=0; i < numBuses; i++)
    {
cout << "2" << endl;
        Bus* curB = &ckt->buses[i];
cout << "3" << endl;
        string filename;
cout << "4" << endl;
        filename = plotdir + curB->name + ".svg";
cout << "5" << endl;
        CreateBusPlot(false, curB->id, filename.c_str());
cout << "6" << endl;
    }
cout << "7" << endl;






}



void CreateBusPlot(bool all, int busid, const char* fileName)
{

    int layoutOffsetX, layoutOffsetY, layoutWidth, layoutHeight;
    int numGCs, numRows, numCols, numLayers, GW, GH;
    int xoffset, yoffset, offset;
    int llx, lly, urx, ury, curl, l1, l2;
    int col, row; 
    int wireid;
    bool isVertical;
    //char* fileName = plotall.c_str();

#ifdef DEBUG
    printf("Current Bus %d\n\n", busid);
#endif



    layoutOffsetX = -1*ckt->originX;
    layoutOffsetY = -1*ckt->originY;
    layoutWidth = ckt->width;
    layoutHeight = ckt->height;


#ifdef DEBUG
    printf("Layout (%8d %8d) (%8d %8d)\n", layoutOffsetX, layoutOffsetY, layoutOffsetX + layoutWidth, layoutOffsetY + layoutHeight);
    
#endif

    Dimensions dimensions(layoutWidth, layoutHeight);
    Document doc(fileName, Layout(dimensions, Layout::BottomLeft, 1));

    Color colors[]
        = { Color::Red, Color::Orange, Color::Yellow, Color::Green, Color::Blue, Color::Purple, Color::Black };
        
        //= { Color::Aqua, Color::Black, Color::Blue, Color::Brown, Color::Cyan, Color::Fuchsia,
        //    Color::Green, Color::Lime, Color::Magenta, Color::Orange, Color::Purple, Color::Red,
        //    Color::Silver, Color::White, Color::Yellow };

    // White space    
    //llx = layoutOffsetX;
    //lly = layoutOffsetY;
    //urx = layoutOffsetX + layoutWidth;
    //ury = layoutOffsetY + layoutHeight;
    llx = 0;
    lly = 0;
    urx = layoutWidth;
    ury = layoutHeight;

    Polygon border(Color::White, Stroke(5,Color::Black));
    border << Point(llx,lly) << Point(llx, ury) << Point(urx,ury) << Point(urx, lly);
    doc << border;
    //doc << Rectangle(Point(0,0), layoutWidth, layoutHeight, Color::White);


    // Grid3D
    br::Grid3D* grid = &rou->grid;
    numRows = grid->numRows;
    numCols = grid->numCols;
    numLayers = grid->numLayers;
    GW = grid->GCELL_WIDTH;
    GH = grid->GCELL_HEIGHT;


    // Grid3D lines
    for(col=0; col<numCols; col++)
    {
        xoffset = grid->GetOffset_x(col);//offsetxs[col];
        llx = xoffset + layoutOffsetX;
        urx = xoffset + layoutOffsetX;
        //lly = layoutOffsetY;
        //ury = layoutOffsetY + layoutHeight;
        lly = 0;
        ury = layoutHeight;

        doc << Line(Point(llx,lly), Point(urx,ury), Stroke(5, Color::Black));
#ifdef DEBUG_GLOBAL
        printf("Line (%8d %8d) (%8d %8d)\n", llx, lly, urx, ury);
#endif
    
    }

    for(row=0; row<numRows; row++)
    {
        yoffset = grid->GetOffset_y(row);//offsetys[row];
        //llx = layoutOffsetX;
        //urx = layoutOffsetX + layoutWidth;
        llx = 0;
        urx = layoutWidth;
        lly = yoffset + layoutOffsetY;
        ury = yoffset + layoutOffsetY;

        doc << Line(Point(llx,lly), Point(urx,ury), Stroke(5, Color::Black));
#ifdef DEBUG_GLOBAL
        printf("Line (%8d %8d) (%8d %8d)\n", llx, lly, urx, ury);
#endif
    }

    int pinid, bitid, segid, viaid, trackid;
    int numBuses, numPins, numBits, numSegs, numWires, numVias, numTracks;
    int centerX, centerY;
    int textOffsetX, textOffsetY;
    int GCllx, GClly, GCurx, GCury;
    int x1, x2, y1, y2, l;
    bool target;
    string content;

    numBuses = ckt->buses.size();
    numBits = ckt->bits.size();
    numPins = ckt->pins.size();
    numSegs = rou->segs.size();
    numWires = rou->wires.size();
    numVias = rou->vias.size();
    numTracks = ckt->tracks.size();

    if(show_track)
    {
        for(trackid=0; trackid < numTracks; trackid++)
        {
            br::Container* curct = &rou->rtree.containers[trackid];
            curl = curct->l;
            for(auto& it : curct->segs)
            {
                llx = (int)(bg::get<0,0>(it) +0.5) + layoutOffsetX;
                lly = (int)(bg::get<0,1>(it) +0.5) + layoutOffsetY;
                urx = (int)(bg::get<1,0>(it) +0.5) + layoutOffsetX;
                ury = (int)(bg::get<1,1>(it) +0.5) + layoutOffsetY;
                doc << Line(Point(llx, lly), Point(urx, ury), Stroke(10,colors[curl]));
            }

        }

    }

    if(maze)
    {
     
        if(!all){
            br::StTree* curtree = rou->rsmt[rou->rsmt.treeID[busid]];
            for(auto& gcellid : curtree->gcells)
            {
                llx = grid->llx(gcellid) + layoutOffsetX;
                lly = grid->lly(gcellid) + layoutOffsetY;
                urx = grid->urx(gcellid) + layoutOffsetX;
                ury = grid->ury(gcellid) + layoutOffsetY;
                curl = rou->grid[gcellid]->l;
                Polygon poly(Fill(colors[curl], 0.3), Stroke(5, Color::Black));
                poly << Point(llx,lly) << Point(llx,ury) << Point(urx, ury) << Point(urx, lly);
                doc << poly;
            }
        }   

    }
    else
    {

        
        for(int i=0; i < numBuses; i++)
        {
            target = (i == busid)?true:false;
            if(!all && !target) continue;   
            br::StTree* curtree = rou->rsmt[rou->rsmt.treeID[busid]];
            for(auto segid : curtree->segs)
            {
                br::Segment* curS = &rou->segs[segid];
                br::Bus* curB = &ckt->buses[rou->seg2bus[curS->id]];
                GCllx = min(curS->x1, curS->x2);
                GClly = min(curS->y1, curS->y2);
                GCurx = max(curS->x1, curS->x2);
                GCury = max(curS->y1, curS->y2);
                curl = curS->l;
                int g1 = grid->GetIndex(GCllx, GClly, curS->l);
                int g2 = grid->GetIndex(GCurx, GCury, curS->l);
                llx = grid->llx(g1) + layoutOffsetX;
                lly = grid->lly(g1) + layoutOffsetY;
                urx = grid->urx(g2) + layoutOffsetX;
                ury = grid->ury(g2) + layoutOffsetY;
                //llx = max(llx, layoutOffsetX);
                //lly = max(lly, layoutOffsetY);
                //urx = min(urx, layoutOffsetX + layoutWidth);
                //ury = min(ury, layoutOffsetY + layoutHeight);
                //llx = max(llx, 0); //layoutOffsetX);
                //lly = max(lly, 0); //layoutOffsetY);
                //urx = min(urx, layoutWidth);
                //ury = min(ury, layoutHeight);

                content = curB->name + " BW[" + to_string(curB->numBits) + "]";
                textOffsetX = (int)(1.0*(urx-llx)/2 + 0.5);
                textOffsetY = (int)(1.0*(ury-lly)/2 + 0.5);


                Polygon poly(Fill(colors[curl], 0.3), Stroke(5, Color::Black));
                poly << Point(llx,lly) << Point(llx,ury) << Point(urx, ury) << Point(urx, lly);
                doc << poly;


                //doc << Rectangle(Point(llx,lly), (urx-llx), (ury-lly), colors[curl]);
#ifdef DEBUG
                printf("Rect (%8d %8d) (%8d %8d) -> Segment (%d %d) (%d %d)\n", llx, lly, urx, ury, GCllx, GClly, GCurx, GCury);
#endif
                //doc << Text(Point(textOffsetX, textOffsetY), content, Fill(), Font(300));
            }


            for(auto& it : ckt->buses[i].bits)
            {
                bitid = it;
                for(auto& p : ckt->bits[bitid].paths)
                {
                    if(p.via)
                    {
                        centerX = p.x[0] + layoutOffsetX;
                        centerY = p.y[0] + layoutOffsetY;
                        curl = p.l;
                        doc << Circle(Point(centerX, centerY), 150.0, colors[curl], Stroke(10, Color::Black));
                    }
                    else
                    {
                        llx = p.x[0] + layoutOffsetX;
                        lly = p.y[0] + layoutOffsetY;
                        urx = p.x[1] + layoutOffsetX;
                        ury = p.y[1] + layoutOffsetY;
                        curl = p.l;
                        int width = ckt->buses[i].width[curl];


                        if(llx == urx)
                        {
                            llx -= (int)(1.0*width/2);
                            urx += (int)(1.0*width/2);
                        }

                        if(lly == ury)
                        {
                            lly -= (int)(1.0*width/2);
                            ury += (int)(1.0*width/2);
                        }

                        Polygon poly(colors[curl], Stroke(10, Color::Black));
                        poly << Point(llx,lly) << Point(llx,ury) << Point(urx, ury) << Point(urx, lly);
                        doc << poly;
                    }

                }
            }

            /*
            for(auto wireid : curtree->wires)
            {
                br::Wire* curWire = &rou->wires[wireid];
                llx = curWire->x1 + layoutOffsetX;
                lly = curWire->y1 + layoutOffsetY;
                urx = curWire->x2 + layoutOffsetX;
                ury = curWire->y2 + layoutOffsetY;
                curl = curWire->l; //ckt->layerHashMap[curPin->layer];   

                if(llx == urx)
                {
                    llx -= (int)(1.0*curWire->width/2);
                    urx += (int)(1.0*curWire->width/2);
                }

                if(lly == ury)
                {
                    lly -= (int)(1.0*curWire->width/2);
                    ury += (int)(1.0*curWire->width/2);
                }

                Polygon poly(colors[curl], Stroke(10, Color::Black));
                poly << Point(llx,lly) << Point(llx,ury) << Point(urx, ury) << Point(urx, lly);
                doc << poly;
                //doc << Rectangle(Point(llx,lly), (urx-llx), (ury-lly), colors[curl]);
#ifdef DEBUG_WIRE
                printf("Rect (%8d %8d) (%8d %8d)\n", llx, lly, urx, ury);
#endif

            }
            */

        }

    }
    // Bus

    for(pinid=0; pinid < numPins; pinid++)
    {
         
        br::Pin* curPin = &ckt->pins[pinid];
        br::Bit* curBit = &ckt->bits[ckt->bitHashMap[curPin->bitName]];
        br::Bus* curB = &ckt->buses[ckt->busHashMap[curBit->busName]];
        target = (curB->id == busid)?true:false;
        if(!target && !all) continue;   
        
        llx = curPin->llx + layoutOffsetX;
        lly = curPin->lly + layoutOffsetY;
        urx = curPin->urx + layoutOffsetX;
        ury = curPin->ury + layoutOffsetY;
        llx = max(llx, 0); //layoutOffsetX);
        lly = max(lly, 0); //layoutOffsetY);
        urx = min(urx, layoutWidth); //layoutOffsetX + layoutWidth);
        ury = min(ury, layoutHeight); //layoutOffsetY + layoutHeight);
        
        curl = curPin->l; //ckt->layerHashMap[curPin->layer];   
        content = curPin->bitName;
        size_t found1 = content.find_last_of('<');
        size_t found2 = content.find_last_of('>');
        content = content.substr(0, found1) + "_" + content.substr(found1+1, found2);

        textOffsetX = (int)(1.0*(urx-llx)/2 + 0.5);
        textOffsetY = (int)(1.0*(ury-lly)/2 + 0.5);


        Polygon poly(colors[curl], Stroke(5, Color::Black));
        poly << Point(llx,lly) << Point(llx,ury) << Point(urx, ury) << Point(urx, lly);
        doc << poly;
        //doc << Rectangle(Point(llx,lly), (urx-llx), (ury-lly), colors[curl]);
#ifdef DEBUG_PIN
        printf("Rect (%8d %8d) (%8d %8d)\n", llx, lly, urx, ury);
#endif
        doc << Text(Point(textOffsetX, textOffsetY), content, Fill(), Font(300));
    }


    // Category
    //int coffsetx, coffsety;
    //int cwidth, cheight;

#ifdef DEBUG
    printf("\n\n\n");
#endif

    
    doc.save();
}
