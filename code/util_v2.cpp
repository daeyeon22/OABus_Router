#include "circuit.h"
#include "util.h"
#include "func.h"
#include "route.h"

#define _DEBUG


using namespace svg;
namespace br = OABusRouter;

static string plotdir = "./plot/";
static string plotall = "./plot/bus_all.svg";
static bool show_track = true;
static bool maze = false;
static double scale = 1.0;

void OABusRouter::Router::create_plot(const char* benchName)
{

    int i, numBuses;
    numBuses = ckt->buses.size();


    for(i=0; i < numBuses; i++)
    {
        Bus* curB = &ckt->buses[i];
        string filename;
        filename = plotdir + benchName + "/" +  curB->name + ".svg";
        create_bus_plot(false, curB->id, filename.c_str());
    }

    string filename = plotdir + benchName + "/" + "bus_all.svg";
    create_bus_plot(true, 0, filename.c_str());
}



void create_bus_plot(bool all, int busid, const char* fileName)
{

    int layoutOffsetX, layoutOffsetY, layoutWidth, layoutHeight;
    int numGCs, numRows, numCols, numLayers, GW, GH;
    int xoffset, yoffset, offset;
    int llx, lly, urx, ury, curl, l1, l2;
    int col, row; 
    int wireid;
    double stroke_width = 5.0;
    double circle_radius = 200;

    bool isVertical;
    //char* fileName = plotall.c_str();

#ifdef DEBUG_UTIL
    printf("Current Bus %d\n\n", busid);
#endif



    layoutOffsetX = -1*ckt->originX;
    layoutOffsetY = -1*ckt->originY;
    layoutWidth = ckt->width;
    layoutHeight = ckt->height;

    stroke_width *= sqrt(sqrt(layoutWidth*layoutHeight) / 10000);
    circle_radius *= sqrt(sqrt(layoutWidth*layoutHeight) / 10000);




#ifdef DEBUG_UTIL
    printf("Layout (%8d %8d) (%8d %8d)\n", layoutOffsetX, layoutOffsetY, layoutOffsetX + layoutWidth, layoutOffsetY + layoutHeight);
    
#endif

    Dimensions dimensions(scale*layoutWidth, scale*layoutHeight);
    Document doc(fileName, Layout(dimensions, Layout::BottomLeft, scale));

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

    Polygon border(Color::White, Stroke(stroke_width,Color::Black));
    border << Point(llx,lly) << Point(llx, ury) << Point(urx,ury) << Point(urx, lly);
    doc << border;
    //doc << Rectangle(Point(0,0), layoutWidth, layoutHeight, Color::White);

    int obsid, pinid, bitid, segid, viaid, trackid;
    int numObs, numBuses, numPins, numBits, numSegs, numWires, numVias, numTracks;
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
    numObs = ckt->obstacles.size();

    for(obsid=0; obsid < numObs; obsid++)
    {
        br::Obstacle* obs = &ckt->obstacles[obsid];
        llx = obs->llx + layoutOffsetX;
        lly = obs->lly + layoutOffsetY;
        urx = obs->urx + layoutOffsetX;
        ury = obs->ury + layoutOffsetY;
        Polygon poly(Fill(colors[obs->l], 0.7), Stroke(stroke_width, Color::Red));
        poly << Point(llx,lly) << Point(llx,ury) << Point(urx, ury) << Point(urx, lly);
        doc << poly;

    }   


    if(show_track)
    {
        for(trackid=0; trackid < numTracks; trackid++)
        {
            br::Interval* interval = rou->rtree_t.get_interval(trackid);
            
            //br::Container* curct = &rou->rtree.containers[trackid];
            curl = interval->l; //curct->l;
            for(auto& it : interval->segs)
            {
                llx = (int)(bg::get<0,0>(it) +0.5) + layoutOffsetX;
                lly = (int)(bg::get<0,1>(it) +0.5) + layoutOffsetY;
                urx = (int)(bg::get<1,0>(it) +0.5) + layoutOffsetX;
                ury = (int)(bg::get<1,1>(it) +0.5) + layoutOffsetY;
                doc << Line(Point(llx, lly), Point(urx, ury), Stroke(2*stroke_width,colors[curl]));
            }

        }

    }
       
    for(int i=0; i < numBuses; i++)
    {
        target = (i == busid)?true:false;
        if(!all && !target) continue;   

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
                    //doc << Circle(Point(centerX, centerY), circle_radius, colors[curl], Stroke(2*stroke_width, Color::Black));
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

                    Polygon poly(colors[curl], Stroke(2*stroke_width, Color::Black));
                    poly << Point(llx,lly) << Point(llx,ury) << Point(urx, ury) << Point(urx, lly);
                    doc << poly;
                }

            }
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


        Polygon poly(colors[curl], Stroke(stroke_width, Color::Black));
        poly << Point(llx,lly) << Point(llx,ury) << Point(urx, ury) << Point(urx, lly);
        doc << poly;
        //doc << Rectangle(Point(llx,lly), (urx-llx), (ury-lly), colors[curl]);
#ifdef DEBUG_UTIL
        printf("Rect (%8d %8d) (%8d %8d)\n", llx, lly, urx, ury);
#endif
        //doc << Text(Point(textOffsetX, textOffsetY), content, Fill(), Font(300));
    }


    // Category
    //int coffsetx, coffsety;
    //int cwidth, cheight;

#ifdef DEBUG_UTIL
    printf("\n\n\n");
#endif

    
    doc.save();
}
