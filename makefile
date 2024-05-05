CXX=clang++-9
CXXFLAGS=-std=c++2a -g -Werror -Wsign-conversion # c++20
VALGRIND_FLAGS=-v --leak-check=full --show-leak-kinds=all  --error-exitcode=99


SOURCES=graph/Graph.cpp algorithms/Algorithms.cpp
OBJECTS=$(subst .cpp,.o,$(SOURCES)) # replace .cpp with .o in SOURCES

.PHONY: run clean test graph algorithms

PROG=main

run: $(PROG) graph algorithms
	./$<


$(PROG): $(PROG).o $(OBJECTS)
	$(CXX) $(CXXFLAGS) $^ -o $@

test: 
	make -C tests test


graph:
	make -C graph all


algorithms:
	make -C algorithms all


%.o: %.cpp 
	$(CXX) $(CXXFLAGS) --compile $< -o $@

clean:
	make -C graph clean
	make -C algorithms clean
	make -C tests clean
	rm -f *.o test core main