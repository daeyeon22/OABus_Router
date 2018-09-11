#ifndef RTREE_H
#define RTREE_H
#include <vector>
#include <set>
#include <climits>
#include <sparsehash/dense_hash_map>
#include <boost/icl/interval_set.hpp>
#include <boost/icl/interval_base_map.hpp>
#include <boost/icl/interval_map.hpp>
#include <boost/geometry.hpp>

#define HORIZONTAL 222
#define VERTICAL 111 
#define PINTYPE -1212
#define OBSTACLE -1232
#define NOT_ASSIGN 3391

using namespace std;
using google::dense_hash_map;
namespace bi = boost::icl;
namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

// For Interval tree library
typedef bi::interval_map<int,set<int>> IntervalMapT;
typedef bi::interval_set<int> IntervalSetT;
typedef bi::interval<int> IntervalT;
typedef bi::discrete_interval<int> DiscreteIntervalT;
// Boost geometries
typedef bg::model::point<float,2, bg::cs::cartesian> PointBG;
typedef bg::model::segment<PointBG> SegmentBG;
typedef bg::model::box<PointBG> BoxBG;
typedef bgi::rtree<pair<PointBG,int>, bgi::rstar<16>> PointRtree;
typedef bgi::rtree<pair<SegmentBG,int>, bgi::rstar<16>> SegRtree;
typedef bgi::rtree<pair<BoxBG,int>, bgi::rstar<16>> BoxRtree;


namespace OABusRouter
{
    enum QueryMode
    {
        Intersects,
        Overlaps,
        Covered,
        Within,
        Disjoint
    };

    
    struct Interval
    {
        typedef SegmentBG seg;
        typedef PointBG pt;
        typedef BoxBG box;

        int trackid;
        int offset;
        int l;
        int width;
        bool vertical;

        IntervalSetT empty;

        vector<seg> segs;
        vector<int> elems;
    
        Interval()
        {}

        Interval(const Interval& i) :
            trackid(i.trackid),
            offset(i.offset),
            l(i.l),
            width(i.width),
            vertical(i.vertical),
            empty(i.empty),
            segs(i.segs),
            elems(i.elems)
        {}

    };

    struct PinRtree
    {
        typedef SegmentBG seg;
        typedef PointBG pt;
        typedef BoxBG box;

        vector<box> elems;
        vector<BoxRtree> rtree;
        dense_hash_map<int,int> elem2bus;
        dense_hash_map<int,int> elem2bit;

        PinRtree() {}

        PinRtree(int numlayers)
        {
            rtree = vector<BoxRtree>(numlayers);
            elem2bus.set_empty_key(INT_MAX);
            elem2bit.set_empty_key(INT_MAX);
        }

        BoxRtree* operator [] (int l)
        {
            return &rtree[l];
        }

        void remove_pins(vector<int> &pins);
        int num_diff_bus_between_twopins(int busid, int p1, int p2, int l);
    };

    struct BitRtree
    {
        typedef SegmentBG seg;
        typedef PointBG pt;
        typedef BoxBG box;
       
        int elemindex;
        vector<box> elems;
        vector<BoxRtree> rtree;
        dense_hash_map<int,int> elem2type;
        dense_hash_map<int,int> elem2pin;
        dense_hash_map<int,int> elem2wire;

        BitRtree()
        {}


        BitRtree(int numlayers)
        {
            elemindex = 0;
            rtree = vector<BoxRtree>(numlayers);
            elem2type.set_empty_key(INT_MAX);
            elem2pin.set_empty_key(INT_MAX);
            elem2wire.set_empty_key(INT_MAX);
        }

        BitRtree(const BitRtree& br):
            elemindex(br.elemindex),
            elems(br.elems),
            rtree(br.rtree),
            elem2type(br.elem2type),
            elem2pin(br.elem2pin),
            elem2wire(br.elem2wire)
        {}

        bool short_violation(int x[], int y[], int l, set<int>& except1, set<int>& except2);
    };

    struct SegmentRtree
    {
        typedef SegmentBG seg;
        typedef PointBG pt;
        typedef BoxBG box;

        vector<box> elems;
        vector<BoxRtree> rtree;

        dense_hash_map<int,int> elem2bus;
        SegmentRtree() {}

        SegmentRtree(int numlayers)
        {
            rtree = vector<BoxRtree>(numlayers);
            elem2bus.set_empty_key(INT_MAX);
        }

        BoxRtree* operator [] (int l)
        {
            return &rtree[l];
        }


    };

    struct TrackRtree
    {
        typedef SegmentBG seg;
        typedef PointBG pt;
        typedef BoxBG box;


        int elemindex;
        vector<SegRtree> rtree;
        vector<Interval> tracks;
        dense_hash_map<int,int> elem2track;
        dense_hash_map<int,int> elem2wire;


        TrackRtree()
        {}


        TrackRtree(int numlayers, int numtracks)
        {
            elemindex = 0;
            rtree = vector<SegRtree>(numlayers);
            tracks = vector<Interval>(numtracks);
            elem2track.set_empty_key(INT_MAX);
            elem2wire.set_empty_key(INT_MAX);
        }
        
        TrackRtree(const TrackRtree& tr) :
            elemindex(tr.elemindex),
            rtree(tr.rtree),
            tracks(tr.tracks),
            elem2track(tr.elem2track),
            elem2wire(tr.elem2wire)
        {}

        SegRtree* operator [] (int l)
        {
            return &rtree[l];
        }

        Interval* get_interval(int trackid)
        {
            return &tracks[trackid];
        }

        SegRtree* get_rtree(int l)
        {
            return &rtree[l];
        }


        
        // getter
        int get_trackid(int e);
        int get_width(int e);
        int get_width_t(int t);
        int get_layer(int e);
        int get_layer_t(int t);
        int get_direction(int e);
        int get_direction_t(int t);
        int get_offset(int e);
        int get_offset_t(int t);
        bool is_vertical(int e);
        bool is_vertical_t(int t);

        bool insert_element(int trackid, int x1, int y1, int x2, int y2, int l, bool remove);
        bool insert_element(int trackid, int x[], int y[], int l, bool remove);
        
        //template <typename A, typename B>
        void get_intersection(int t1, int t2, int &x, int &y);
        void query(int mode, box geo, int l, vector<pair<seg,int>>& queries);
        void query(int mode, seg geo, int l, vector<pair<seg,int>>& queries);
        void query(int mode, box geometry, int lower, int upper, vector<pair<seg,int>> &queries);
        void query(int mode, seg geometry, int lower, int upper, vector<pair<seg,int>> &queries);
        
    };


    struct ObstacleRtree
    {
        typedef SegmentBG seg;
        typedef PointBG pt;
        typedef BoxBG box;
        
        // design bound
        int db[4];
        vector<BoxRtree> rtree;

        ObstacleRtree()
        {}

        ObstacleRtree(int numlayers)
        {
            rtree = vector<BoxRtree>(numlayers);
        }

        ObstacleRtree(const ObstacleRtree& ort):
            rtree(ort.rtree)
        {
            db[0] = ort.db[0];
            db[1] = ort.db[1];
            db[2] = ort.db[2];
            db[3] = ort.db[3];
        }

        BoxRtree* operator [] (int l)
        {
            return &rtree[l];
        }

        BoxRtree* get_rtree(int l)
        {
            return &rtree[l];
        }

        // member functions
        bool insert_obstacle(int bitid, int x[], int y[], int l, bool remove);
        bool compactness(int numbits, int mx[], int my[], int x, int y, int l1, int l2, int align, int dir, int width, int spacing);
        bool spacing_violations(int bitid, int x[], int y[], int l, int width, int spacing, bool vertical);
        bool spacing_violations_ndr(int bitid, int x[], int y[], int l);
        bool short_violation(int bitid, int wirex[], int wirey[], int wl, int tarx[], int tary[], int tl);
        int num_spacing_violations(int bitid, int x[], int y[], int l, int width, int spacing, bool vertical);
        int num_spacing_violations_ndr(int bitid, int x[], int y[], int l);
    };
};
#endif




