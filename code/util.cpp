#include "util.h"
#include "circuit.h"


using namespace svg;
namespace BR = OABusRouter;
string plotDir = "./plot/";

OABusRouter::Point OABusRouter::Rect::center()
{
    int cx = (int)((this->ll.x + this->ur.x)/2 + 0.5);
    int cy = (int)((this->ll.y + this->ur.y)/2 + 0.5);
    
    return OABusRouter::Point(cx,cy);
}


void OABusRouter::Circuit::GenPlot()
{
    for(int i=0; i < this->buses.size() ; i++)
    {
        OABusRouter::Bus* bus = &this->buses[i];
        if(bus->numPinShapes > 3)
        {
            string fileName =plotDir +  bus->name + ".svg";
            GenBusPlot(bus->id , fileName.c_str());
        }


    }





}


void GenBusPlot(int busid, const char* fileName)
{
 
    int leftBnd = ckt->designBoundary.ll.x;
    int rightBnd = ckt->designBoundary.ur.x;
    int bottomBnd = ckt->designBoundary.ll.y;
    int topBnd = ckt->designBoundary.ur.y;


    Dimensions dimensions((rightBnd - leftBnd), (topBnd - bottomBnd));
    Document doc(fileName, Layout(dimensions, Layout::BottomLeft));
    Polygon border(Stroke(1, Color::Red));

    // SVG boundary
    border << Point(0,0) << Point(dimensions.width, 0) << Point(dimensions.width, dimensions.height) 
        << Point(0, dimensions.height);
    doc << border;


    // BUS Shape
    BR::Bus* bus = &ckt->buses[busid];   
    for(int i=0; i < bus->bits.size() ; i++){
        BR::Bit* bit = &ckt->bits[bus->bits[i]];

        int deg = 0;
        int x[MAXD], y[MAXD];
        Tree flutetree;
        bool genTree = (i==0)?true:false;
        
        
        for(int j=0; j < bit->pins.size() ; j++){
            BR::Pin* pin = &ckt->pins[bit->pins[j]];
            Polygon pinShape(Color::Orange, Stroke(3, Color::Black));
            pinShape << Point(pin->boundary.ll.x, pin->boundary.ll.y) 
                << Point(pin->boundary.ur.x, pin->boundary.ll.y)
                << Point(pin->boundary.ur.x, pin->boundary.ur.y)
                << Point(pin->boundary.ll.x, pin->boundary.ur.y);

            doc << pinShape;
            if(genTree)
            {
                OABusRouter::Point center = pin->boundary.center();
                x[deg] = center.x;
                y[deg] = center.y;
                deg++;
            }
        
        }

        // Create flute tree
        
        if(genTree)
        {
            flutetree = flute(deg, x, y, ACCURACY);
            for(int d=0; d<flutetree.deg; d++){
                doc << Circle(Point(flutetree.branch[d].x, flutetree.branch[d].y), 10, Fill(Color::Blue), Stroke(3, Color::Black));
            }
        }
        
    }

    doc.save();    
}











