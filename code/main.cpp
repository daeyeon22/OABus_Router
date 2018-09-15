
#include "circuit.h"
#include "route.h"

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



bool OABusRouter::Circuit::should_stop()
{
    double elapse_time = measure.elapse_time();
    double runtime_limit = (double)runtime * 60;

    if(elapse_time > 0.9*runtime_limit)
    {
        printf("\n");
        printf("[INFO] elapse time over 80% runtime limit\n");
        printf("[INFO] Runtime limit   : %.2f\n", runtime_limit);
        printf("[INFO] Elapse time     : %.2f\n", elapse_time);
        printf("\n");
        return true;
    }
    else
        return false;
}


int main(int argc, char** argv){
    
    measure.start_clock();
    
    cout << "================================================================" <<endl;
    cout << "    ICCAD 2018 Contest on Obstacle-aware Bus Routing            " <<endl;
    cout << "    Authors : Daeyeon Kim, SangGi Do                            " <<endl;
    cout << "    Advisor : Seok-Hyeong Kang                                  " <<endl;
    cout << "================================================================" <<endl;

    char* inputFileName;
    char* outputFileName;
    string benchName;
    string logDirName;
    string outDirName;
    string lefName;
    string defName;
    int numThreads;

    for(int i=1; i < argc; i++){
        if(i+1 != argc){
            if(strncmp(argv[i], "-input", 6) == 0){
                inputFileName = argv[++i];   
                string tmp = inputFileName;
                tmp = tmp.substr(0, tmp.find_last_of("."));
                benchName = tmp.substr(tmp.find_last_of("/")+1, tmp.size());

                
                //size_t found1 = tmp.find_last_of("/");
                //size_t found2 = tmp.find_last_of(".");
                //cout << found1 << " " << found2 << endl;
                //benchName = tmp.substr(0,found2);//tmp.substr(found1+1,found2);
            }
            if(strncmp(argv[i], "-threads", 8) == 0){
                numThreads = atoi(argv[++i]);
            }
            if(strncmp(argv[i], "-output", 7) == 0){

                outputFileName = argv[++i];
                string tmp = outputFileName;
                tmp = tmp.substr(0, tmp.find_last_of("/"));
                outDirName = tmp;
                logDirName = "../log" + tmp.substr(tmp.find_last_of("/"), tmp.size());
            }
        }
    }


    cout << "< Argument Report >" << endl;
    cout << "Input      : " << inputFileName << endl;
    cout << "Output     : " << outputFileName << endl;
    cout << "Bench      : " << benchName << endl;
    cout << "Out Dir    : " << outDirName << endl;
    cout << "Log Dir    : " << logDirName << endl;
    cout << "# threads  : " << numThreads << endl;
    cout << endl;

    if(!ckt->read_iccad2018(inputFileName)){
        cout << "Fail to read " << inputFileName << endl;
    }

    cout << "[INFO] start Initialize" << endl;
    ckt->initialize();
    cout << "[INFO] start route all" << endl;
    rou->route_all();
    
    cout << "[INFO] start Create Path" << endl;
    ckt->create_path();
    //cout << "[INFO] start Create Plot" << endl;
    //rou->create_plot(benchName.c_str());
    cout << "[INFO] start Write out file" << endl;
    ckt->out_write(outputFileName);
    //cout << "[INFO] start Write def & lef file" << endl;
    //lefName = logDirName + "/" + benchName + ".lef";
    //defName = logDirName + "/" + benchName + ".def";
    //ckt->lef_write(lefName);
    //ckt->def_write(defName);

    //rou->penalty_cost();

    cout << "[INFO] End program" << endl;

    measure.stop_clock("All");
    measure.print_clock();

//    cout << "[INFO] start Write lef & def file" << endl;
//    lefName = logDirName + "/" + benchName + ".lef";
//    defName = logDirName + "/" + benchName + ".def";
//    ckt->lef_write(lefName);
//    ckt->def_write(defName);
    
    return 0;
}


