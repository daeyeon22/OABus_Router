
#include "circuit.h"
#include "route.h"
//#include "util.h"
//#define DEBUG_MP

using namespace std;


// Static variables
OABusRouter::Circuit* OABusRouter::Circuit::instance = nullptr;
OABusRouter::Circuit* OABusRouter::Circuit::shared(){
    if (instance == nullptr) instance = new Circuit();
    return instance;
}

OABusRouter::Router* OABusRouter::Router::instance = nullptr;
OABusRouter::Router* OABusRouter::Router::shared(){
    if(instance == nullptr) instance = new Router();
    return instance;
}
//


int main(int argc, char** argv){
    
    cout << "================================================================" <<endl;
    cout << "    ICCAD 2018 Contest on Obstacle-aware Bus Routing            " <<endl;
    cout << "    Authors : Daeyeon Kim, Seungwon Kim                         " <<endl;
    cout << "    Advisor : Seok-Hyeong Kang                                  " <<endl;
    cout << "================================================================" <<endl;

    char* inputFileName;
    char* outputFileName;
    int numThreads;

    for(int i=1; i < argc; i++){
        if(i+1 != argc){
            if(strncmp(argv[i], "-input", 6) == 0){
                inputFileName = argv[++i];   
            }
            if(strncmp(argv[i], "-threads", 8) == 0){
                numThreads = atoi(argv[++i]);
            }
            if(strncmp(argv[i], "-output", 7) == 0){
                outputFileName = argv[++i];
            }
        }
    }


    cout << "Input     : " << inputFileName << endl;
    cout << "Output    : " << outputFileName << endl;
    cout << "# threads : " << numThreads << endl;

    if(!ckt->read_iccad2018(inputFileName)){
        cout << "Fail to read " << inputFileName << endl;
    }

#ifdef DEBUG_MP
    for(int i=0; i < ckt->multipins.size(); i++) {
        OABusRouter::MultiPin* mp = &ckt->multipins[i];
        cout << mp->id << endl;
        cout << ckt->buses[mp->busid].name << endl;
        
        cout << ckt->layers[mp->l].name << endl;
        cout << mp->pins.size() << " " << ckt->buses[mp->busid].numBits << endl;
        cout << " - - - - - - - - " << endl;
    }
    //exit(0);
#endif

   
    ckt->Printall();

    //exit(0);

    cout << "Initialize" << endl;
    ckt->Init();
    rou->InitGrid3D();
    
    
    //cout << "Initialize" << endl;
    //ckt->Init();
    
    cout << "Generate Backbone" << endl;
    rou->GenBackbone();
    //ckt->GenBackbone_v2();
   
    //ckt->RoutingPoint();
    //cout << "Generate Plot file" << endl;
    //ckt->GenPlot();

    cout << "Topology Mapping 3D" << endl;
    rou->TopologyMapping3D();


    //cout << "Solve ILP" << endl;
    //rou->SolveILP();
    //<< endl;
    //ckt->InitRoutingDirection();

    cout << "Create Clips" << endl;
    rou->CreateClips();


    cout << "Solve ILP v2" << endl;
    rou->SolveILP_v2();


    cout << "Post Global Routing" << endl;
    rou->PostGlobalRouting();

    //exit(0);
    cout << "TrackAssign" << endl;
    rou->TrackAssign();


    cout << "Create Via" << endl;
    rou->CreateVia();

    cout << "Mapping multipin to segment, pin to wire" << endl;
    //rou->MappingMultipin2Seg();
    //rou->MappingPin2Wire();

    cout << "Create Plot" << endl;
    rou->Plot();

    
    cout << "Write def & lef file" << endl;
    //ckt->def_write();
    //ckt->lef_write();


<<<<<<< HEAD
=======

>>>>>>> a9d10c4578ebfe6fef220d80e7d14bc33473dacf
    cout << "End program" << endl;
    return 0;
}


