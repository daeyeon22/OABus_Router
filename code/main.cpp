
#include "circuit.h"
#include "route.h"
#include "mymeasure.h"

using namespace std;
static CMeasure measure;

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
    string benchName;
    int numThreads;
    measure.start_clock();



    
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
            }
        }
    }


    cout << "Input      : " << inputFileName << endl;
    cout << "Output     : " << outputFileName << endl;
    cout << "Bench      : " << benchName << endl;
    cout << "# threads  : " << numThreads << endl;

    if(!ckt->read_iccad2018(inputFileName)){
        cout << "Fail to read " << inputFileName << endl;
    }

    cout << "Initialize" << endl;
    ckt->initialize();
    //rou->initialize();
    cout << "Route all" << endl;
    rou->route_all();
    cout << "Create Path" << endl;
    ckt->create_path();
    cout << "Create Plot" << endl;
    rou->create_plot(benchName.c_str());
    cout << "Write def & lef file" << endl;
    ckt->out_write(outputFileName);

    cout << "End program" << endl;

    measure.stop_clock("All");
    measure.print_clock();

    return 0;
}


