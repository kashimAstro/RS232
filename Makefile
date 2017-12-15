all:
	mkdir -p bin
	g++ -Wall -o bin/example-led example-led/main.cpp -Isrc/ -std=c++11
	g++ -Wall -o bin/example-button example-button/main.cpp -Isrc/ -std=c++11
