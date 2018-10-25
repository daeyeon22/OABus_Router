
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
#ifdef REPORT
        cout << "Fail to read " << inputFileName << endl;
#endif
    }

#ifdef REPORT
    cout << "[INFO] start Initialize" << endl;
#endif
    ckt->initialize();

    cout << "[INFO] construct rtree" << endl;
    rou->construct_rtree(); 

    cout << "[INFO] route all" << endl;
    rou->route_all();


    cout << "[INFO] start Create Path" << endl;
    ckt->create_path();
    
    cout << "[INFO] start Create Plot" << endl;
    rou->create_plots(benchName.c_str());


    cout << "[INFO] start Write out file" << endl;
    ckt->out_write(outputFileName);

    cout << "[INFO] start Write def & lef file" << endl;
    lefName = logDirName + "/" + benchName + ".lef";
    defName = logDirName + "/" + benchName + ".def";
    ckt->lef_write(lefName);
    ckt->def_write(defName);   

    /*
#ifdef REPORT
    cout << "[INFO] start route all" << endl;
#endif
    rou->route_all();
    
#ifdef REPORT
    cout << "[INFO] start Create Path" << endl;
#endif
    ckt->create_path();

#ifdef REPORT
    cout << "[INFO] start Create Plot" << endl;
    rou->create_plot(benchName.c_str());
#endif

#ifdef REPORT
    cout << "[INFO] start Write out file" << endl;
#endif
    ckt->out_write(outputFileName);

#ifdef REPORT
    cout << "[INFO] start Write def & lef file" << endl;
    lefName = logDirName + "/" + benchName + ".lef";
    defName = logDirName + "/" + benchName + ".def";
    ckt->lef_write(lefName);
    ckt->def_write(defName);
#endif
    */
    cout << "[INFO] End program" << endl;

    measure.stop_clock("All");
    measure.print_clock();

#ifdef REPORT
    print_welapse_time();
#endif

    return 0;
}


