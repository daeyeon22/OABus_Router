#include "circuit.h"

#define DEBUG

using namespace std;
using namespace OABusRouter;

void Circuit::def_write() {
    return def_write("output.def");
}

void Circuit::def_write(string file_name) {

    ofstream dot_def(file_name.c_str());

    if(!dot_def.good()) {
        cerr << "def_writer :: cannot open '" << file_name << "' for writing. " << endl;
        #ifdef DEBUG
            exit(0);
        #endif
    }

    // INITIAL DEF description
    dot_def << "VERSION 5.7 ;" << endl;
    dot_def << "DIVIDERCHAR \"\/\" ;" << endl;
    dot_def << "BUSBITCHARS \"\[\]\" ;" << endl;
    //dot_def << "DESIGN ;" << endl;
    dot_def << "UNITS DISTANCE MICRONS 1000 ;" << endl;
    dot_def << endl;
    dot_def << "DIEAREA ;" << endl;
    dot_def << endl;
    dot_def << " ;" << endl;
    dot_def << " ;" << endl;




    return;
}
