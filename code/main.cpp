
#include "circuit.h"

using namespace std;

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



    OABusRouter::Circuit ckt;

    if(!ckt.read_iccad2018(inputFileName)){
        cout << "Fail to read " << inputFileName << endl;
    }


    return 0;
}

