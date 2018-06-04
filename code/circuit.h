#ifndef __CIRCUIT__
#define __CIRCUIT__ 0
#include <iostream>
#include <string>
#include <fstream>
#include <vector>
#include <map>
#include <cmath>
#include <climits>
#include <cfloat>
#include <algorithm>
#include <assert.h>
#include <cstdlib>
#include <cstring>

#include <sparsehash/dense_hash_map>

#define INIT_STR "INITSTR"
#define VERTICAL 111
#define HORIZONTAL 222

using namespace std;
using google::dense_hash_map;


namespace OABusRouter
{
    struct Point
    {
        int x;
        int y;

        // Constructor
        Point() : x(INT_MAX), y(INT_MAX) {}
        Point(int ix, int iy) : x(ix), y(iy) {}
        Point(Point& pt) : x(pt.x), y(pt.y) {}
        
        void print();
    };
    
    struct Rect
    {
        Point ll;
        Point ur;

        // Constructor
        Rect()
        {
            ll = Point();
            ur = Point();
        }

        Rect(Point ill, Point iur) : 
            ll(ill), 
            ur(iur) {}

        Rect(Rect& rt) : 
            ll(rt.ll), 
            ur(rt.ur) {}

        void print();

    };

    
    struct Layer
    {
        int id;
        int direction;
        int spacing;
        string name;
        Rect boundary;

        Layer() : 
            id(INT_MAX), 
            direction(INT_MAX), 
            spacing(INT_MAX), 
            name(INIT_STR) 
        {

        }

        Layer(const Layer& lr) :
            id(lr.id),
            direction(lr.direction),
            spacing(lr.spacing),
            name(lr.name)
        {

        }

        void print();
    };


    struct Track
    {
        int id;
        int width;
        string layer;
        Point ll;
        Point ur;

        Track() : 
            id(INT_MAX), 
            width(INT_MAX), 
            layer(INIT_STR) {}

        Track(const Track& tr) :
            id(tr.id),
            width(tr.width),
            layer(tr.layer)
        {
            ll = tr.ll;
            ur = tr.ur;
        }

        void print();
    };


    struct Pin
    {
        int id;
        string bitName;
        string layer;
        Rect boundary;

        Pin() : 
            id(INT_MAX), 
            bitName(INIT_STR), 
            layer(INIT_STR) {}

        Pin(const Pin& p) :
            id(p.id),
            bitName(p.bitName),
            layer(p.layer)
        {
            boundary = p.boundary;
        }

        void print();
    };

    struct Bit
    {
        int id;
        string name;
        string busName;
        vector<int> pins;

        Bit() : 
            id(INT_MAX), 
            name(INIT_STR), 
            busName(INIT_STR) {}

        Bit(const Bit& b) :
            id(b.id),
            name(b.name),
            busName(b.busName)
        {
            pins.insert(pins.end(), b.pins.begin(), b.pins.end());
        }

        void print();
    };

    struct Bus
    {
        int id;
        int numBits;
        int numPinShapes;
        string name;

        vector<int> bits;
        dense_hash_map<string,int> width;
        //HashMap;

        Bus() : 
            id(INT_MAX), 
            numBits(INT_MAX), 
            numPinShapes(INT_MAX), 
            name(INIT_STR)
        {
            width.set_empty_key(INIT_STR);
        }

        Bus(const Bus& b) :
            id(b.id),
            numBits(b.numBits),
            numPinShapes(b.numPinShapes),
            name(b.name)
        {
            bits = b.bits;
            width = b.width;
        }

        void print();
    };


    struct Obstacle
    {
        int id;
        string layer;
        Rect boundary;

        Obstacle() : 
            id(INT_MAX), 
            layer(INIT_STR) {}

        Obstacle(const Obstacle& obs) :
            id(obs.id),
            layer(obs.layer)
        {
            boundary = obs.boundary;   
        }

        void print();

    };

    class Circuit
    {
      public:
        // Parameters
        int runtime;
        int alpha;
        int beta;
        int delta;
        int gamma;
        int epsilon;
        Rect designBoundary;


        // Objects
        vector<Layer> layers;
        vector<Track> tracks;
        vector<Bus> buses;
        vector<Obstacle> obstacles;
        vector<Bit> bits;
        vector<Pin> pins;

        // Hash Map
        dense_hash_map<string,int> bitHashMap;
        dense_hash_map<string,int> busHashMap;
        dense_hash_map<string,int> layerHashMap;

        // Initializer
        Circuit() :
            runtime(INT_MAX), 
            alpha(INT_MAX),
            beta(INT_MAX),
            delta(INT_MAX),
            gamma(INT_MAX),
            epsilon(INT_MAX)
        {
            bitHashMap.set_empty_key(INIT_STR);
            busHashMap.set_empty_key(INIT_STR);
            layerHashMap.set_empty_key(INIT_STR);
        }


        
        // Getter
        Bit* getBit(string& name);
        Bus* getBus(string& name);
        Layer* getLayer(string* name);

        // Parser
        bool read_iccad2018(char* fileName);
        bool getParam(char* fileName);
        bool getLayerInfo(char* fileName);
        bool getTrackInfo(char* fileName);
        bool getBusInfo(char* fileName);
        bool getObstacleInfo(char* fileName);


    };


};




#endif
