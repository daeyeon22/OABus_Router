
#include "circuit.h"
#include "route.h"
#define REPORT



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


void print_welapse_time()
{
    double elapse_time = measure.elapse_time();
    printf("[INFO] wall elapse time %.f s\n", elapse_time);
}


int main(int argc, char** argv){
    
    measure.start_clock();
    
    cout << "================================================================" <<endl;
    cout << "    ICCAD 2018 Contest on Obstacle-aware Bus Routing            " <<endl;
    cout << "    Authors : Daeyeon Kim, SangGi Do, SungYun Lee               " <<endl;
    cout << "    Advisor : Seok-Hyeong Kang                                  " <<endl;
    cout << "================================================================" <<endl;
    
    
    char* inputFileName;
    char* outputFileName;
    string benchName;
    string logDirName;
    string outDirName;
    string lefName;
    string defName;
    int numThreads = 4;

    inputFileName = argv[1];
    string tmp = inputFileName;
    tmp = tmp.substr(0, tmp.find_last_of("."));
    benchName = tmp.substr(tmp.find_last_of("/")+1,tmp.size());

    outputFileName = argv[2];
    tmp = outputFileName;
    tmp = tmp.substr(0, tmp.find_last_of("/"));
    outDirName = tmp;
    logDirName = "../log" + tmp.substr(tmp.find_last_of("/"), tmp.size());

#ifdef REPORT
    cout << "< Argument Report >" << endl;
    cout << "Input      : " << inputFileName << endl;
    cout << "Output     : " << outputFileName << endl;
    cout << "Bench      : " << benchName << endl;
    cout << "Out Dir    : " << outDirName << endl;
    cout << "Log Dir    : " << logDirName << endl;
    cout << "# threads  : " << numThreads << endl;
    cout << endl;
#endif

    if(!ckt->read_iccad2018(inputFileName)){
        cout << "Fail to read " << inputFileName << endl;
    }

    cout << "[INFO] start Initialize" << endl;
    ckt->initialize();

    measure.stop_clock("Start initialize");
    
    cout << "[INFO] construct rtree" << endl;
    rou->construct_rtree(); 

    measure.stop_clock("Construct Rtree");
    
    cout << "[INFO] route all" << endl;
    rou->route_all();
    measure.stop_clock("Route All");

    cout << "[INFO] ripup and reroute" << endl;
    rou->ripup_reroute();

    measure.stop_clock("Ripup And Reroute");

    cout << "[INFO] start Create Path" << endl;
    ckt->create_path();
    
    measure.stop_clock("Create Path");
    
    cout << "[INFO] start Create Plot" << endl;
    //rou->create_plots(benchName.c_str());

    measure.stop_clock("Plot Generation");


    cout << "[INFO] start Write out file" << endl;
    ckt->out_write(outputFileName);

    cout << "[INFO] start Write def & lef file" << endl;
    lefName = logDirName + "/" + benchName + ".lef";
    defName = logDirName + "/" + benchName + ".def";
    ckt->lef_write(logDirName, benchName);
    ckt->def_write(logDirName, benchName);   

    measure.stop_clock("Write Out/Lef/Def Files");

    measure.stop_clock("All");
    measure.print_clock();

#ifdef REPORT
    print_welapse_time();
#endif

    cout << "[INFO] End program" << endl;
    return 0;
}


