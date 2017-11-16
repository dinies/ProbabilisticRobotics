EIGEN_INCLUDE=/usr/include/eigen3
CXXOPTS=-O3 -DNDEBUG -I$(EIGEN_INCLUDE)

kd_tree:	kd_tree.cpp
	g++ -o $@ $(CXXOPTS) $^

clean:
	rm -rf *.o kd_tree *~
