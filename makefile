.PHONY: run clean test

CXX=clang++-9
CXXFLAGS=-std=c++2a -g # c++20
VALGRIND_FLAGS=-v --leak-check=full --show-leak-kinds=all  --error-exitcode=99


SOURCES=Graph.cpp Algorithms.cpp
OBJECTS=$(subst .cpp,.o,$(SOURCES)) # replace .cpp with .o in SOURCES
TEST=Test.cpp TestCounter.cpp
TEST_OBJECTS=$(subst .cpp,.o,$(TEST))

run: demo
	./$^

demo: Demo.o $(OBJECTS)
	$(CXX) $(CXXFLAGS) $^ -o demo

test: $(TEST_OBJECTS) $(OBJECTS)
	$(CXX) $(CXXFLAGS) $^ -o test

Algorithms.o: Algorithms.cpp Algorithms.hpp
	$(CXX) $(CXXFLAGS) --compile $< -o $@

Graph.o: Graph.cpp Graph.hpp
	$(CXX) $(CXXFLAGS) --compile $< -o $@

%.o: %.cpp 
	$(CXX) $(CXXFLAGS) --compile $< -o $@

clean:
	rm -f *.o demo test core