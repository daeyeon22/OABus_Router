EXE = iccad18obr
CXX = g++

SYSTEM = x86-64_linux
LIBFORMAT = static_pic

CXXOPTS = -m64 -O3 -fPIC -fno-strict-aliasing -fexceptions -fpermissive -DNDEBUG -DIL_STD 
#CFLAGS = $(CXXOPTS) $(LIBINCLUDE) -fopenmp -std=c++11

CFLAGS = -std=c++11 -O3 -fpermissive -g -ggdb -DNDEBUG -DIL_STD $(LIBINCLUDE)
CCFLAGS = -O3 -I.

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


$(EXE): func.o main.o util.o parser.o flute.o flute_mst.o dl.o mst2.o neighbors.o dist.o heap.o err.o
	$(CXX) $(DEBUG) $(CXXOPTS) $^ -o $@ $(LINKCFLAGS) 
#2> out.log

func.o: func.cpp func.h
	$(CXX) $(CFLAGS) -c -o $@ $<  

main.o: main.cpp circuit.h
	$(CXX) $(CFLAGS) -c -o $@ $<

parser.o: parser.cpp circuit.h func.h
	$(CXX) $(CFLAGS) -c -o $@ $<

neighbors.o: neighbors.c neighbors.h
	$(CXX) $(CCFLAGS) -c -o $@ $< 

mst2.o: mst2.c mst2.h
	$(CXX) $(CCFLAGS) -c -o $@ $< 

dl.o: dl.c dl.h
	$(CXX) $(CCFLAGS) -c -o $@ $< 
#$(CXX) $(CCFLAGS) -c -o dl.o dl.c

err.o: err.c err.h
	$(CXX) $(CCFLAGS) -c -o $@ $< 

heap.o: heap.c heap.h
	$(CXX) $(CCFLAGS) -c -o $@ $< 

dist.o: dist.c dist.h
	$(CXX) $(CCFLAGS) -c -o $@ $< 

flute.o: flute.c flute.h
	$(CXX) $(CCFLAGS) -c -o $@ $< 
#gcc -O3 -I. -c -o flute.o flute.c

flute_mst.o: flute_mst.c flute.h
	$(CXX) $(CCFLAGS) -c -o $@ $< 
#gcc -O3 -I. -c -o flute_mst.o flute_mst.c

#util.o: util.cpp circuit.h flute.h util.h func.h
#	$(CXX) $(CFLAGS) -c -o $@ $<  

init.o: init.cpp circuit.h func.h
	$(CXX) $(CFLAGS) -c -o $@ $<  

#RSMT.o: RSMT.cpp flute.h circuit.h global.h
#	$(CXX) $(CFLAGS) -c -o $@ $<  



.PHONY: clean
clean:
	/bin/rm -f *.o 2> /dev/null; /bin/rm -f $(EXE) 2> /dev/null; /bin/rm -f *.log;



