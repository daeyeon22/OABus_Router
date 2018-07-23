#include "circuit.h"
#include "util.h"
#include "func.h"
#include "route.h"


using namespace svg;
namespace br = OABusRouter;

static string plotdir = "./plot/";
static string plotall = "./plot/bus_all.svg";


void OABusRouter::Router::Plot()
{

    cout << "Create plots..." << endl;
    int i, numBuses;
    numBuses = ckt->buses.size();

    for(i=0; i < numBuses; i++)
    {
        Bus* curB = &ckt->buses[i];
        string filename;
        filename = plotdir + curB->name + ".svg";
        CreateBusPlot(false, curB->id, filename.c_str());
    }






}


void CreateBusPlot(bool all, int busid, const char* fileName)
{

    int layoutOffsetX, layoutOffsetY, layoutWidth, layoutHeight;
    int numGCs, numRows, numCols, numLayers, GW, GH;
    int xoffset, yoffset;
    int llx, lly, urx, ury, curl;
    int col, row; 
    int wireid;

    //char* fileName = plotall.c_str();


    layoutOffsetX = ckt->originX;
    layoutOffsetY = ckt->originY;
    layoutWidth = ckt->width;
    layoutHeight = ckt->height;


#ifdef DEBUG
    printf("Layout (%8d %8d) (%8d %8d)\n", layoutOffsetX, layoutOffsetY, layoutWidth, layoutHeight);
#endif

    Dimensions dimensions(layoutWidth, layoutHeight);
    Document doc(fileName, Layout(dimensions, Layout::BottomLeft, 1));

    Color colors[]
        = { Color::Aqua, Color::Black, Color::Blue, Color::Brown, Color::Cyan, Color::Fuchsia,
            Color::Green, Color::Lime, Color::Magenta, Color::Orange, Color::Purple, Color::Red,
            Color::Silver, Color::White, Color::Yellow };

    // White space    
    llx = layoutOffsetX;
    lly = layoutOffsetY;
    urx = layoutOffsetX + layoutWidth;
    ury = layoutOffsetY + layoutHeight;

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
        llx = xoffset;
        urx = xoffset;
        lly = layoutOffsetY;
        ury = layoutOffsetY + layoutHeight;

        doc << Line(Point(llx,lly), Point(urx,ury), Stroke(5, Color::Black));
#ifdef DEBUG
        printf("Line (%8d %8d) (%8d %8d)\n", llx, lly, urx, ury);
#endif
    
    }

    for(row=0; row<numRows; row++)
    {
        yoffset = grid->GetOffset_y(row);//offsetys[row];
        llx = layoutOffsetX;
        urx = layoutOffsetX + layoutWidth;
        lly = yoffset;
        ury = yoffset;

        doc << Line(Point(llx,lly), Point(urx,ury), Stroke(5, Color::Black));
#ifdef DEBUG
        printf("Line (%8d %8d) (%8d %8d)\n", llx, lly, urx, ury);
#endif
    }

    int pinid, bitid, segid, viaid;
    int numBuses, numPins, numBits, numSegs, numWires, numVias;
    int centerX, centerY;
    int textOffsetX, textOffsetY;
    int GCllx, GClly, GCurx, GCury;
    bool target;
    string content;

    numBuses = ckt->buses.size();
    numBits = ckt->bits.size();
    numPins = ckt->pins.size();
    numSegs = rou->segs.size();
    numWires = rou->wires.size();
    numVias = rou->vias.size();

    //
    for(segid=0; segid < numSegs; segid++)
    {
        br::Segment* curS = &rou->segs[segid];
        br::Bus* curB = &ckt->buses[rou->seg2bus[curS->id]];
        target = (curB->id == busid)?true:false;
        if(!all && !target) continue;   
        GCllx = min(curS->x1, curS->x2);
        GClly = min(curS->y1, curS->y2);
        GCurx = max(curS->x1, curS->x2);
        GCury = max(curS->y1, curS->y2);

        llx = grid->GetOffset_x(GCllx) + layoutOffsetX;
        lly = grid->GetOffset_y(GClly) + layoutOffsetY;
        urx = grid->GetOffset_x(GCurx) + GW + layoutOffsetX;
        ury = grid->GetOffset_y(GCury) + GH + layoutOffsetY;
        llx = max(llx, layoutOffsetX);
        lly = max(lly, layoutOffsetY);
        urx = min(urx, layoutOffsetX + layoutWidth);
        ury = min(ury, layoutOffsetY + layoutHeight);
        curl = curS->l;
        
        content = curB->name + " BW[" + to_string(curB->numBits) + "]";
        textOffsetX = (int)(1.0*(urx-llx)/2 + 0.5);
        textOffsetY = (int)(1.0*(ury-lly)/2 + 0.5);
       

        Polygon poly(Fill(colors[curl], 0.3), Stroke(5, Color::Black));
        poly << Point(llx,lly) << Point(llx,ury) << Point(urx, ury) << Point(urx, lly);
        doc << poly;
        

        //doc << Rectangle(Point(llx,lly), (urx-llx), (ury-lly), colors[curl]);
#ifdef DEBUG
        printf("Rect (%8d %8d) (%8d %8d)\n", llx, lly, urx, ury);
#endif
        //doc << Text(Point(textOffsetX, textOffsetY), content, Fill(), Font(300));

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
        llx = max(llx, layoutOffsetX);
        lly = max(lly, layoutOffsetY);
        urx = min(urx, layoutOffsetX + layoutWidth);
        ury = min(ury, layoutOffsetY + layoutHeight);
        
        curl = curPin->l; //ckt->layerHashMap[curPin->layer];   
        content = curPin->bitName;
        textOffsetX = (int)(1.0*(urx-llx)/2 + 0.5);
        textOffsetY = (int)(1.0*(ury-lly)/2 + 0.5);


        Polygon poly(colors[curl], Stroke(5, Color::Black));
        poly << Point(llx,lly) << Point(llx,ury) << Point(urx, ury) << Point(urx, lly);
        doc << poly;
        //doc << Rectangle(Point(llx,lly), (urx-llx), (ury-lly), colors[curl]);
#ifdef DEBUG
        printf("Rect (%8d %8d) (%8d %8d)\n", llx, lly, urx, ury);
#endif
        //doc << Text(Point(textOffsetX, textOffsetY), content, Fill(), Font(300));
    }


    // Wire
    for(wireid=0; wireid < numWires; wireid++)
    {
        br::Wire* curWire = &rou->wires[wireid];
        target = (curWire->busid == busid)? true : false;
        if(!target && !all) continue;

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
#ifdef DEBUG
        printf("Rect (%8d %8d) (%8d %8d)\n", llx, lly, urx, ury);
#endif
    }


    // Via
    for(viaid=0; viaid < numVias; viaid++)
    {
        br::Via* curV = &rou->vias[viaid];
        
        target = (rou->via2bus[viaid] == busid)? true : false;
        if(!target && !all) continue;

        centerX = curV->x;
        centerY = curV->y;
        curl = curV->l1;
        double diameter = 1000;

        doc << Circle(Point(centerX, centerY), diameter, Fill(colors[curl], 1.0), Stroke(10, Color::Black));
//#ifdef DEBUG
        printf("Circle (%8d %8d) Diameter %d\n", centerX, centerY, (int)diameter);
//#endif

    }
    // Category
    //int coffsetx, coffsety;
    //int cwidth, cheight;
    doc.save();
}
