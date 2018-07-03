#include "circuit.h"
#include "flute.h"
#include "util.h"
#include "func.h"

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
        if(bus->numPinShapes >= 3)
        {
            string fileName1 = plotDir + bus->name + "v1.svg";
            string fileName2 = plotDir + bus->name + "v2.svg";
            GenBusPlot(bus->id, fileName1.c_str());
            GenBusPlot_v2(bus->id , fileName2.c_str());

        }


    }

}



void GenBusPlot_v2(int busid, const char* fileName)
{
    BR::Bus* bus = &ckt->buses[busid];
    int layoutOffsetX = -1*bus->llx;
    int layoutOffsetY = -1*bus->lly;
    double layoutWidth = 1.0*(bus->urx - bus->llx);// + 1.0*layoutMargin*4;
    double layoutHeight = 1.0*(bus->ury - bus->lly);// + 1.0*layoutMargin*4;
    //int whiteSpaceWidth = bus->urx - bus->llx + layoutMargin*2;
    //int whiteSpaceHeight = bus->ury - bus->lly + layoutMargin*2;
    

    Dimensions dimensions(layoutWidth, layoutHeight);
    Document doc(fileName, Layout(dimensions, Layout::BottomLeft, 1));
    
    Color colors[]
        = { Color::Aqua, Color::Black, Color::Blue, Color::Brown, Color::Cyan, Color::Fuchsia,
            Color::Green, Color::Lime, Color::Magenta, Color::Orange, Color::Purple, Color::Red,
            Color::Silver, Color::White, Color::Yellow };

    // White space    
    doc << Rectangle(Point(0,0), layoutWidth, layoutHeight, Color::White);


    // Tracks
    int numBits = bus->numBits;
    int numPinShapes = bus->numPinShapes;
    int ublind = INT_MIN, lblind = INT_MAX;
    for(int i=0; i < numPinShapes; i++)
    {
        BR::Bit* curBit = &ckt->bits[bus->bits[0]];
        int indL = ckt->layerHashMap[ckt->pins[curBit->pins[i]].layer];
        lblind = min(lblind, indL);
        ublind = max(ublind, indL);
    }

    while(lblind <= ublind)
    {
        BR::Layer* curL = &ckt->layers[lblind++];
        int lowerB, upperB;
        if(curL->is_vertical())
        {
            lowerB = GetLowerBound(curL->trackOffsets, bus->llx);// - layoutMargin);
            upperB = GetUpperBound(curL->trackOffsets, bus->urx);// + layoutMargin);
        }else{
            lowerB = GetLowerBound(curL->trackOffsets, bus->lly);// - layoutMargin);
            upperB = GetUpperBound(curL->trackOffsets, bus->ury);// + layoutMargin);
        }

        while(lowerB < upperB)
        {
            
            int llx, lly;
            int urx, ury;
            if(curL->is_vertical())
            {
                llx = curL->trackOffsets[lowerB] + layoutOffsetX;
                urx = curL->trackOffsets[lowerB] + layoutOffsetX;
                lly = 0;//+layoutOffsetY;
                ury = layoutHeight;//+whiteSpaceHeight;// + layoutOffsetY;
            }else{
                lly = curL->trackOffsets[lowerB] + layoutOffsetY;
                ury = curL->trackOffsets[lowerB] + layoutOffsetY;
                llx = 0;//layoutOffsetX;
                urx = layoutWidth;// + layoutOffsetX;
            }
            doc << Line(Point(llx, lly), Point(urx, ury), Stroke(5, colors[curL->id]));
            printf("Track (%d %d) (%d %d)\n", llx, lly, urx, ury);
            lowerB++;
        }
        
    }

    bool isDone = false;
    for(int i=0; i < numBits; i++){
        BR::Bit* curBit = &ckt->bits[bus->bits[i]];
        for(int j=0; j < numPinShapes; j++)
        {
            BR::Pin* curPin = &ckt->pins[curBit->pins[j]];
            int indl = ckt->layerHashMap[curPin->layer];
            int x1 = curPin->boundary.ll.x + layoutOffsetX;
            int x2 = curPin->boundary.ur.x + layoutOffsetX;
            int y1 = curPin->boundary.ll.y + layoutOffsetY;
            int y2 = curPin->boundary.ur.y + layoutOffsetY;

            doc << Rectangle(Point(x1,y1), (x2-x1), (y2-y1), colors[indl]);
        }

        //if(!isDone)
        //{
            BR::StTree *sttree = &ckt->stTrees[ckt->stTreeHashMap[curBit->id]];
            // Nodes
            for(int n=0; n < sttree->numNodes; n++)
            {
                BR::TreeNode* node = &sttree->nodes[n];
                int objectX = node->x + layoutOffsetX;
                int objectY = node->y + layoutOffsetY;

                if(node->isPin)
                {
                    doc << Circle(Point(objectX,objectY), 100, Fill(Color::Blue), Stroke(3, Color::Black));
                }else{
                    doc << Circle(Point(objectX,objectY), 100, Fill(Color::Red), Stroke(3, Color::Black));
                }
                //doc << Text(Point(x, y), content, Fill(), Font(300)); 
            }
            // Edges
            for(int e=0; e < sttree->numEdges; e++)
            {
                BR::TreeEdge* edge = &sttree->edges[e];
                BR::TreeNode* n1 = &sttree->nodes[edge->n1];
                BR::TreeNode* n2 = &sttree->nodes[edge->n2];

                int x1 = n1->x + layoutOffsetX;
                int y1 = n1->y + layoutOffsetY;
                int x2 = n2->x + layoutOffsetX;
                int y2 = n2->y + layoutOffsetY;

                if(x1 != x2 && y1 != y2)
                {
                    doc << Line(Point(x1,y1), Point(x2,y1), Stroke(50, Color::Red));
                    doc << Line(Point(x2,y1), Point(x2,y2), Stroke(50, Color::Red));
                }
                else
                {
                    doc << Line(Point(x1,y1), Point(x2,y2), Stroke(50, Color::Red));
                }
            }



            //isDone = true;
        //}
        //


    }
    doc.save();


}

void GenBusPlot(int busid, const char* fileName)
{
    int leftBnd = ckt->designBoundary.ll.x;
    int rightBnd = ckt->designBoundary.ur.x;
    int bottomBnd = ckt->designBoundary.ll.y;
    int topBnd = ckt->designBoundary.ur.y;


    //readLUT();
    //printf("Design Boundary (%d %d) (%d %d)\n", ckt->designBoundary.ll.x, ckt->designBoundary.ll.y, ckt->designBoundary.ur.x, ckt->designBoundary.ur.y);
   

    // BUS Shape
    BR::Bus* bus = &ckt->buses[busid];   
    int numBits = bus->bits.size();
    int layoutOffsetX[numBits];
    int layoutOffsetY[numBits];
    int layoutMargin = 2000; 
    int layoutWidth = (bus->urx - bus->llx);
    int layoutHeight = (bus->ury - bus->lly);
    int designOffsetX = ckt->designBoundary.ll.x - bus->llx;
    int designOffsetY = ckt->designBoundary.ll.y - bus->lly;

    Dimensions dimensions((double)(layoutWidth * numBits), (double)(layoutHeight + layoutMargin*2));
    Document doc(fileName, Layout(dimensions, Layout::BottomLeft, 1));
   

    // Layout
    for(int i=0; i < numBits; i++)
    {
        layoutOffsetX[i] = layoutWidth*i + layoutMargin*(i+1);// + designOffsetX;
        layoutOffsetY[i] = 4000;// + designOffsetY;
        
        Polygon layBorder(Fill(Color::White), Stroke(100, Color::Black));
        
        int llx = layoutOffsetX[i];
        int lly = layoutOffsetY[i];
        int urx = layoutWidth  + layoutOffsetX[i];
        int ury = layoutHeight + layoutOffsetY[i] + 4000;
        
        printf("Layout (%d %d) (%d %d)\n", llx, lly, urx, ury); 
        layBorder 
            << Point(llx, lly - layoutMargin) << Point(llx, ury + layoutMargin) 
            << Point(urx, ury + layoutMargin) << Point(urx, lly - layoutMargin);
        doc << layBorder;

        int textOffsetX = (int)(1.0*(urx - llx)/2 + 0.5) + layoutOffsetX[i];
        int textOffsetY = 1000;

        string text = to_string(i);
        doc << Text(Point(textOffsetX, textOffsetY), text, Fill(), Font(500)); 

    }



    for(int i=0; i < 1; i++){
        BR::Bit* bit = &ckt->bits[bus->bits[i]];


        std::string content = std::to_string(i);
        BR::StTree *sttree = &ckt->stTrees[ckt->stTreeHashMap[bit->id]];
        // Nodes
        for(int n=0; n < sttree->numNodes; n++)
        {
            BR::TreeNode* node = &sttree->nodes[n];
            int objectX = node->x + layoutOffsetX[i] + designOffsetX;
            int objectY = node->y + layoutOffsetY[i] + designOffsetY;
            
            if(node->isPin)
            {
                doc << Circle(Point(objectX,objectY), 400, Fill(Color::Blue), Stroke(3, Color::Black));
            }else{
                doc << Circle(Point(objectX,objectY), 400, Fill(Color::Red), Stroke(3, Color::Black));
            }
            //doc << Text(Point(x, y), content, Fill(), Font(300)); 
        }

        // Edges
        for(int e=0; e < sttree->numEdges; e++)
        {
            BR::TreeEdge* edge = &sttree->edges[e];
            BR::TreeNode* n1 = &sttree->nodes[edge->n1];
            BR::TreeNode* n2 = &sttree->nodes[edge->n2];

            int x1 = n1->x;//min(n1->x, n2->x);
            int x2 = n2->x;//max(n1->x, n2->x);
            int y1 = n1->y;//min(n1->y, n2->y);
            int y2 = n2->y;//max(n1->y, n2->y);
            //printf("From (%d %d) To (%d %d)\n", x1, y1, x2, y2);

            // For Gcell
            
            // (x1,y1) -> (x1,y2) -> (x2,y2)
            int lowerBoundX = GetLowerBound(ckt->gCellOffsetUX, min(x1,x2));
            int upperBoundX = GetUpperBound(ckt->gCellOffsetUX, max(x1,x2));
            int lowerBoundY = GetLowerBound(ckt->gCellOffsetUY, min(y1,y2));
            int upperBoundY = GetUpperBound(ckt->gCellOffsetUY, max(y1,y2));
            int idxY = GetLowerBound(ckt->gCellOffsetUY,y2);
            int idxX = GetLowerBound(ckt->gCellOffsetUX,x1);

            if(upperBoundX % GCELL_WIDTH != 0){
                upperBoundX++;
            }

            if(upperBoundY % GCELL_WIDTH != 0)
            {
                upperBoundY++;
            }
            
            
            //printf("Bound (%d %d) (%d %d)\n", lowerBoundX, upperBoundX, lowerBoundY, upperBoundY);

            while(lowerBoundY < upperBoundY)
            {
                int urx = ckt->gCellOffsetUX[idxX];
                int ury = ckt->gCellOffsetUY[lowerBoundY];
                int llx = urx - GCELL_WIDTH;//ckt->gCellOffsetLX[lowerBoundX]; //urx - GCELL_WIDTH;
                int lly = ury - GCELL_HEIGHT;//ckt->gCellOffsetLX[lowerBoundY]; // ury - GCELL_HEIGHT;

                //printf("GCELL (%d %d) (%d %d)\n", llx, lly, urx, ury);
                //std::pair<int,int> key = std::make_pair(llx,lly);
                //size_t hkey = GetHashKey(key);
                //if(added.find(hkey) == added.end()){      
                    int objectURX = urx + layoutOffsetX[i] + designOffsetX;
                    int objectURY = ury + layoutOffsetY[i] + designOffsetY;
                    int objectLLX = llx + layoutOffsetX[i] + designOffsetX;
                    int objectLLY = lly + layoutOffsetY[i] + designOffsetY;

                    Polygon gcellBorder(Fill(Color::Yellow), Stroke(1, Color::Black));
                    gcellBorder 
                        << Point(objectLLX, objectLLY) << Point(objectLLX, objectURY) 
                        << Point(objectURX, objectURY) << Point(objectURX, objectLLY);
                    doc << gcellBorder;
                    //added.emplace(hkey);
                //}
                lowerBoundY++;
            }

            //lowerBoundY--;
            while(lowerBoundX < upperBoundX)
            {

                
                int urx = ckt->gCellOffsetUX[lowerBoundX];
                int ury = ckt->gCellOffsetUY[idxY];
                int llx = urx - GCELL_WIDTH;//ckt->gCellOffsetLX[lowerBoundX]; //urx - GCELL_WIDTH;
                int lly = ury - GCELL_HEIGHT;//ckt->gCellOffsetLX[lowerBoundY]; // ury - GCELL_HEIGHT;
                //int llx = ckt->gCellOffsetLX[lowerBoundX]; //urx - GCELL_WIDTH;
                //int lly = ckt->gCellOffsetLX[lowerBoundY]; // ury - GCELL_HEIGHT;
                //printf("GCELL (%d %d) (%d %d)\n", llx, lly, urx, ury);
                //int llx = urx - GCELL_WIDTH;
                //int lly = ury - GCELL_HEIGHT;
                //std::pair<int,int> key = std::make_pair(llx,lly);
                //size_t hkey = GetHashKey(key);
                //if(added.find(hkey) == added.end()){      
                    int objectURX = urx + layoutOffsetX[i] + designOffsetX;
                    int objectURY = ury + layoutOffsetY[i] + designOffsetY;
                    int objectLLX = llx + layoutOffsetX[i] + designOffsetX;
                    int objectLLY = lly + layoutOffsetY[i] + designOffsetY;

                    Polygon gcellBorder(Fill(Color::Yellow), Stroke(1, Color::Black));
                    gcellBorder 
                        << Point(objectLLX, objectLLY) << Point(objectLLX, objectURY) 
                        << Point(objectURX, objectURY) << Point(objectURX, objectLLY);
                    doc << gcellBorder;
                    //added.emplace(hkey);
                //}
                
                lowerBoundX++;
            }





            /*
            if(x1 != x2 && y1 != y2){
                doc << Circle(Point(x1, y2), 1000, Fill(Color::Green), Stroke(3, Color::Black));
                //doc << Text(Point(x1, y2), content, Fill(), Font(100)); 
                //doc << Line(Point(x1, y1), Point(x1, y2), Stroke(5, Color::Black));
                //doc << Line(Point(x1, y2), Point(x2, y2), Stroke(5, Color::Black));
            }else{
                //doc << Line(Point(x1, y1), Point(x2, y2), Stroke(5, Color::Black));
            }
            */
            //cout << endl;
        }

        
        for(int j=0; j < bit->pins.size() ; j++){
            BR::Pin* pin = &ckt->pins[bit->pins[j]];

            int objectURX = pin->boundary.ur.x + layoutOffsetX[i] + designOffsetX;
            int objectURY = pin->boundary.ur.y + layoutOffsetY[i] + designOffsetY;
            int objectLLX = pin->boundary.ll.x + layoutOffsetX[i] + designOffsetX;
            int objectLLY = pin->boundary.ll.y + layoutOffsetY[i] + designOffsetY;
            
            Polygon pinShape(Color::Orange, Stroke(3, Color::Black));
            pinShape 
                << Point(objectLLX, objectLLY) << Point(objectLLX, objectURY) 
                << Point(objectURX, objectURY) << Point(objectURX, objectLLY);

            doc << pinShape;
        }
    }

    doc.save();    
}











