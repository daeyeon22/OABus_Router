EXE = bus_router
CXX = g++

CFLAGS = -std=c++11 -fopenmp -m64 -O2 $(LIBINCLUDE)
#CFLAGS = -std=c++11 -m64 -O3 -mtune=native -fpermissive -fopenmp -g -ggdb -DNDEBUG -DIL_STD $(LIBINCLUDE)

# include library settings..
LIBINCLUDE = -I$(SRCHASH) -I$(BOOST)

# linking flag settings
LINKCFLAGS = $(CFLAGS) -static-libstdc++

# source settings.
SRCHASH 	= /home/tool/cpplib/sparsehash/install/2.0.3/include
BOOST 		= /home/tool/cpplib/boost/install/1.67.0/include

SRC			= func.o parser.o route.o writer.o rtree.o util.o\
			  init.o maze.o cut.o rip.o mymeasure.o main.o topology.o

DEBUG = -g -gstrict-dwarf -gdwarf-2 -pg

$(EXE): $(SRC) 
	$(CXX) $(CFLAGS) $^ -o $@ $(LINKCFLAGS) 
	#$(CXX) $^ -o $@ $(LINKCFLAGS) 

func.o: func.cpp func.h
	$(CXX) $(CFLAGS) -c -o $@ $<  

parser.o: parser.cpp circuit.h func.h
	$(CXX) $(CFLAGS) -c -o $@ $<

route.o: route.cpp route.h circuit.h rtree.h
	$(CXX) $(CFLAGS) -c -o $@ $<  

writer.o: writer.cpp circuit.h route.h
	$(CXX) $(CFLAGS) -c -o $@ $<

rtree.o: rtree.cpp rtree.h circuit.h route.h
	$(CXX) $(CFLAGS) -c -o $@ $<  

util.o: util_v2.cpp circuit.h route.h
	$(CXX) $(CFLAGS) -c -o $@ $<  

init.o: init.cpp circuit.h route.h rtree.h
	$(CXX) $(CFLAGS) -c -o $@ $<  

maze.o: maze.cpp route.h rtree.h
	$(CXX) $(CFLAGS) -c -o $@ $<  

cut.o: cut.cpp route.h
	$(CXX) $(CFLAGS) -c -o $@ $<  

rip.o: rip.cpp route.h
	$(CXX) $(CFLAGS) -c -o $@ $<  
	
mymeasure.o: mymeasure.cpp mymeasure.h
	$(CXX) $(CFLAGS) -c -o $@ $<  

main.o: main.cpp circuit.h
	$(CXX) $(CFLAGS) -c -o $@ $<

topology.o: topology.cpp route.h circuit.h rtree.h
	$(CXX) $(CFLAGS) -c -o $@ $<

.PHONY: clean
clean:
	/bin/rm -f *.o 2> /dev/null; /bin/rm -f $(EXE) 2> /dev/null; /bin/rm -f *.log;


