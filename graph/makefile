CXX=clang++-9
CXXFLAGS=-std=c++2a -g # c++20
VALGRIND_FLAGS=-v --leak-check=full --show-leak-kinds=all  --error-exitcode=99

.PHONY: all clean

all: Graph.o

Graph.o: Graph.cpp Graph.hpp
	$(CXX) $(CXXFLAGS) --compile $< -o $@


clean:
	rm -f *.o
