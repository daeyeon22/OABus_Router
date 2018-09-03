EXE = iccad18obr
CXX = g++

SYSTEM = x86-64_linux
LIBFORMAT = static_pic

CXXOPTS = -m64 -O3 -fPIC -fno-strict-aliasing -fexceptions -fpermissive -DNDEBUG -DIL_STD 
#CFLAGS = $(CXXOPTS) $(LIBINCLUDE) -fopenmp -std=c++11

CFLAGS = -std=c++11 -O3 -fpermissive -g -ggdb -DNDEBUG -DIL_STD $(LIBINCLUDE)
CCFLAGS = -O3 -I.

# include library settings..
LIBINCLUDE = -I$(SRCHASH) -I$(CPLEXDIR) -I$(CONCERTDIR) -I$(BOOST)

# linking flag settings
LINKCFLAGS = $(CFLAGS) -static-libstdc++ $(CPLEXLIBS) $(CONCERTLIBS) 


# source settings.
#SRCHASH 	= /home/tool/sparsehash/include
#BOOST 		= /home/tool/boost/install/1.63.0/include
#CPLEXDIR 	= /home/tool/IBM/ILOG/127/cplex/include
#CONCERTDIR 	= /home/tool/IBM/ILOG/127/concert/include
# source settings.
SRCHASH 	= /home/tool/cpplib/sparsehash/install/2.0.3/include
BOOST 		= /home/tool/cpplib/boost/install/1.63.0/include
CPLEXDIR 	= /home/tool/ibm/install/12.7.0/cplex/include
CONCERTDIR 	= /home/tool/ibm/install/12.7.0/concert/include


# FLUTE #
FLUTEDIR 	= ./flute
FLUTESRC 	= $(FLUTEDIR)/neighbors.o $(FLUTEDIR)/mst2.o \
			$(FLUTEDIR)/dl.o $(FLUTEDIR)/err.o $(FLUTEDIR)/heap.o \
			$(FLUTEDIR)/dist.o $(FLUTEDIR)/flute.o $(FLUTEDIR)/flute_mst.o 

#SRC			= func.o main.o parser.o route.o init.o maze.o\
			  rsmt.o ilp.o util.o rtree.o track.o writer.o\
			  pin_access.o cut.o rip.o mymeasure.o
SRC			= func.o main.o parser.o route.o init.o maze.o\
			  util.o rtree.o writer.o cut.o rip.o mymeasure.o

CPLEXLIBS       = -L$(CPLEXDIR)/../lib/$(SYSTEM)/$(LIBFORMAT) -lilocplex -lcplex
CONCERTLIBS     = -L$(CONCERTDIR)/../lib/$(SYSTEM)/$(LIBFORMAT) -lconcert -lm -pthread -fopenmp -static-libstdc++ -std=c++11

DEBUG = -g -gstrict-dwarf -gdwarf-2 -pg

#
#$(EXE): $(FLUTESRC) $(SRC) 
$(EXE): $(SRC) 
	$(CXX) $(DEBUG) $(CXXOPTS) $^ -o $@ $(LINKCFLAGS) 
#2> out.log
func.o: func.cpp func.h
	$(CXX) $(CFLAGS) -c -o $@ $<  
main.o: main.cpp circuit.h
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

#track.o: track.cpp circuit.h route.h
#	$(CXX) $(CFLAGS) -c -o $@ $<  

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

#rsmt.o: rsmt.cpp route.h circuit.h
#	$(CXX) $(CFLAGS) -c -o $@ $<  

#ilp.o: ilp.cpp ilp.h circuit.h route.h
#	$(CXX) $(CFLAGS) -c -o $@ $<  

#pin_access.o: pin_access.cpp circuit.h route.h
#	$(CXX) $(CFLAGS) -c -o $@ $<


#RSMT.o: RSMT.cpp flute.h circuit.h global.h
#	$(CXX) $(CFLAGS) -c -o $@ $<  


############### FLUTE #################

#flute/neighbors.o: $(FLUTEDIR)/neighbors.c $(FLUTEDIR)/neighbors.h
#	$(CXX) $(CCFLAGS) -c -o $@ $< 

#flute/mst2.o: $(FLUTEDIR)/mst2.c $(FLUTEDIR)/mst2.h
#	$(CXX) $(CCFLAGS) -c -o $@ $< 

#flute/dl.o: $(FLUTEDIR)/dl.c $(FLUTEDIR)/dl.h
#	$(CXX) $(CCFLAGS) -c -o $@ $< 

#flute/err.o: $(FLUTEDIR)/err.c $(FLUTEDIR)/err.h
#	$(CXX) $(CCFLAGS) -c -o $@ $< 

#flute/heap.o: $(FLUTEDIR)/heap.c $(FLUTEDIR)/heap.h
#	$(CXX) $(CCFLAGS) -c -o $@ $< 

#flute/dist.o: $(FLUTEDIR)/dist.c $(FLUTEDIR)/dist.h
#	$(CXX) $(CCFLAGS) -c -o $@ $< 

#flute/flute.o: $(FLUTEDIR)/flute.c $(FLUTEDIR)/flute.h
#	$(CXX) $(CCFLAGS) -c -o $@ $< 

#flute/flute_mst.o: $(FLUTEDIR)/flute_mst.c $(FLUTEDIR)/flute.h
#	$(CXX) $(CCFLAGS) -c -o $@ $< 

########################################

	
.PHONY: clean
clean:
	/bin/rm -f *.o 2> /dev/null; /bin/rm -f $(EXE) 2> /dev/null; /bin/rm -f *.log;



