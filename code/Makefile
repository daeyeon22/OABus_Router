
EXE = iccad18obr
CXX = g++

SYSTEM = x86-64_linux
LIBFORMAT = static_pic

CXXOPTS = -m64 -O3 -fPIC -fno-strict-aliasing -fexceptions -fpermissive -DNDEBUG -DIL_STD 

CFLAGS = -std=c++11 -m64 -O3 -mtune=native -fpermissive -g -ggdb -DNDEBUG -DIL_STD $(LIBINCLUDE)

# include library settings..
LIBINCLUDE = -I$(SRCHASH) -I$(BOOST)

# linking flag settings
LINKCFLAGS = $(CFLAGS) -static-libstdc++

# source settings.
SRCHASH 	= /home/tool/cpplib/sparsehash/install/2.0.3/include
BOOST 		= /home/tool/cpplib/boost/install/1.67.0/include

SRC			= func.o main.o parser.o route.o init.o maze.o\
			  util.o rtree.o writer.o cut.o rip.o mymeasure.o

DEBUG = -g -gstrict-dwarf -gdwarf-2 -pg

$(EXE): $(SRC) 
	$(CXX) $^ -o $@ $(LINKCFLAGS) 
	#$(CXX) $(DEBUG) $(CXXOPTS) $^ -o $@ $(CFLAGS) 

func.o: func.cpp
	$(CXX) $(CFLAGS) -c -o $@ $<  

main.o: main.cpp
	$(CXX) $(CFLAGS) -c -o $@ $<

parser.o: parser.cpp
	$(CXX) $(CFLAGS) -c -o $@ $<

route.o: route.cpp
	$(CXX) $(CFLAGS) -c -o $@ $<  

writer.o: writer.cpp
	$(CXX) $(CFLAGS) -c -o $@ $<

rtree.o: rtree.cpp
	$(CXX) $(CFLAGS) -c -o $@ $<  

util.o: util_v2.cpp 
	$(CXX) $(CFLAGS) -c -o $@ $<  

init.o: init.cpp
	$(CXX) $(CFLAGS) -c -o $@ $<  

maze.o: maze.cpp
	$(CXX) $(CFLAGS) -c -o $@ $<  

cut.o: cut.cpp
	$(CXX) $(CFLAGS) -c -o $@ $<  

rip.o: rip.cpp
	$(CXX) $(CFLAGS) -c -o $@ $<  
	
mymeasure.o: mymeasure.cpp mymeasure.h
	$(CXX) $(CFLAGS) -c -o $@ $<  

.PHONY: clean
clean:
	/bin/rm -f *.o 2> /dev/null; /bin/rm -f $(EXE) 2> /dev/null; /bin/rm -f *.log;


