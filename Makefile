
test_node: Node.cpp
	 g++ -D TEST_NODE -o test Node.cpp
	./test
test_grid: Node.cpp Grid.cpp
	g++ -D TEST_GRID -o testg Node.cpp Grid.cpp
	./testg

