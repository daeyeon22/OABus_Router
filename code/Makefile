EXE = bus_router
CXX = g++
SYSTEM = x86-64_linux
LIBFORMAT = static_pic

CFLAGS = -std=c++11 -fopenmp -m64 -O3 $(LIBINCLUDE)

# include library settings..
LIBINCLUDE = -I$(SRCHASH)/include -I$(BOOST)/include 

# linking flag settings
LINKCFLAGS = $(CFLAGS) -static-libstdc++ $(LIBGOMP)

# source settings.
SRCHASH = /home/tool/cpplib/sparsehash/install/2.0.3
BOOST = /home/tool/cpplib/boost/install/1.67.0
LIBGOMP = -L/home/tool/gcc/install/5.4.0/lib64/libgomp.a -static

SRC			= parser.o tree.o func.o twopin.o local_area.o util.o\
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

#heap.o: heap.cpp heap.h
#	$(CXX) $(CFLAGS) -c -o $@ $<

tree.o: tree.cpp tree.h
	$(CXX) $(CFLAGS) -c -o $@ $<

func.o: func.cpp func.h 
	$(CXX) $(CFLAGS) -c -o $@ $<

twopin.o: twopin.cpp route.h
	$(CXX) $(CFLAGS) -c -o $@ $<

local_area.o: local_area.cpp route.h
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


