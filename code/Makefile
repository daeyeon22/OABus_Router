EXE = bus_router
CXX = g++

CFLAGS = -std=c++11 -fopenmp -m64 -O3 -g -ggdb -DNDEBUG $(LIBINCLUDE)

# include library settings..
LIBINCLUDE = -I$(SRCHASH) -I$(BOOST)

# linking flag settings
LINKCFLAGS = $(CFLAGS) #-static-libstdc++

# source settings.
SRCHASH 	= /home/tool/cpplib/sparsehash/install/2.0.3/include
BOOST 		= /home/tool/cpplib/boost/install/1.67.0/include

SRC			= parser.o tree.o func.o twopin.o util.o\
			  init.o mymeasure.o main.o route.o writer.o 

DEBUG = -g -gstrict-dwarf -gdwarf-2 -pg

$(EXE): $(SRC) 
	$(CXX) $(CFLAGS) $^ -o $@ $(LINKCFLAGS) 

parser.o: parser.cpp circuit.h
	$(CXX) $(CFLAGS) -c -o $@ $<

init.o: init.cpp circuit.h route.h tree.h
	$(CXX) $(CFLAGS) -c -o $@ $<  

mymeasure.o: mymeasure.cpp mymeasure.h
	$(CXX) $(CFLAGS) -c -o $@ $<  

main.o: main.cpp circuit.h
	$(CXX) $(CFLAGS) -c -o $@ $<

tree.o: tree.cpp tree.h
	$(CXX) $(CFLAGS) -c -o $@ $<

func.o: func.cpp func.h 
	$(CXX) $(CFLAGS) -c -o $@ $<

twopin.o: twopin.cpp route.h
	$(CXX) $(CFLAGS) -c -o $@ $<

route.o: route.cpp route.h
	$(CXX) $(CFLAGS) -c -o $@ $<

writer.o: writer.cpp circuit.h
	$(CXX) $(CFLAGS) -c -o $@ $<

util.o: util.cpp util.h
	$(CXX) $(CFLAGS) -c -o $@ $<

.PHONY: clean
clean:
	/bin/rm -f *.o 2> /dev/null; /bin/rm -f $(EXE) 2> /dev/null; /bin/rm -f *.log;


