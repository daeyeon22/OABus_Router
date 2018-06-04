EXE = iccad18obr
CXX = g++
SYSTEM = x86-64_linux
LIBFORMAT = static_pic





CXXOPTS = -m64 -O3 -fPIC -fno-strict-aliasing -fexceptions -fpermissive -DNDEBUG -DIL_STD -Wno-ctor-dtor-privacy
#CFLAGS = $(CXXOPTS) $(LIBINCLUDE) -fopenmp -std=c++11

CFLAGS = -std=c++11 -O3 -fpermissive -g -ggdb -DNDEBUG -DIL_STD -Wno-ctor-dtor-privacy $(LIBINCLUDE)

# include library settings..
LIBINCLUDE = -I$(SRCHASH)/include -I$(CPLEXDIR) -I$(CONCERTDIR) -I$(BOOST)

# linking flag settings
LINKCFLAGS = $(CFLAGS) -static-libstdc++ $(CPLEXLIBS) $(CONCERTLIBS) 

# source settings.
SRCHASH = /home/tool/sparsehash
BOOST = /home/tool/boost/install/1.63.0/include
CPLEXDIR = /home/tool/IBM/ILOG/127/cplex/include
CONCERTDIR = /home/tool/IBM/ILOG/127/concert/include

CPLEXLIBS       = -L$(CPLEXDIR)/../lib/$(SYSTEM)/$(LIBFORMAT) -lilocplex -lcplex
CONCERTLIBS     = -L$(CONCERTDIR)/../lib/$(SYSTEM)/$(LIBFORMAT) -lconcert -lm -pthread -fopenmp -static-libstdc++ -std=c++11

DEBUG = -g -gstrict-dwarf -gdwarf-2
$(EXE): main.o parser.o
	$(CXX) $(DEBUG) $(CXXOPTS) $^ -o $@ $(LINKCFLAGS)

main.o: main.cpp circuit.h
	$(CXX) $(CFLAGS) -c -o $@ $<

parser.o: parser.cpp circuit.h
	$(CXX) $(CFLAGS) -c -o $@ $<


.PHONY: clean
clean:
	/bin/rm -f *.o 2> /dev/null; /bin/rm -f $(EXE) 2> /dev/null; /bin/rm -f *.log;



