# @author Shay Gali
# @link shay.gali@msmail.ariel.ac.il


CXX=clang++-9
CXXFLAGS=-std=c++2a -g -Werror -Wsign-conversion # c++20
VALGRIND_FLAGS=-v --leak-check=full --show-leak-kinds=all --error-exitcode=99


SOURCES= ../graph/Graph.cpp ../algorithms/Algorithms.cpp
OBJECTS=$(subst .cpp,.o,$(SOURCES)) # replace .cpp with .o in SOURCES
TEST=test.cpp TestCounter.cpp
TEST_OBJECTS=$(subst .cpp,.o,$(TEST))

.PHONY: clean test graph algorithms

test: $(TEST_OBJECTS) $(OBJECTS)
	$(CXX) $(CXXFLAGS) $^ -o test
	./test

valgrind: test
	valgrind --tool=memcheck $(VALGRIND_FLAGS) ./test 2>&1 | { egrep "lost| at " || true; }
	
tidy:
	clang-tidy $(TEST) -checks=bugprone-*,clang-analyzer-*,cppcoreguidelines-*,performance-*,portability-*,readability-*,-cppcoreguidelines-pro-bounds-pointer-arithmetic,-cppcoreguidelines-owning-memory --warnings-as-errors=-* --

graph:
	make -C ../graph all

algorithms:
	make -C ../algorithms all

%.o: %.cpp 
	$(CXX) $(CXXFLAGS) --compile $< -o $@

clean:
	make -C ../graph clean
	make -C ../algorithms clean
	rm -f *.o test core