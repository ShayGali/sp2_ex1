# @author Shay Gali
# @link shay.gali@msmail.ariel.ac.il

CXX=clang++-9
CXXFLAGS=-std=c++2a -g -Werror -Wsign-conversion # c++20
VALGRIND_FLAGS=-v --leak-check=full --show-leak-kinds=all  --error-exitcode=99

.PHONY: all clean

all: Algorithms.o

Algorithms.o: Algorithms.cpp Algorithms.hpp
	make -C ../graph all 
	$(CXX) $(CXXFLAGS) --compile $< -o $@

clean:
	rm -f *.o
