solver: main.cpp
	g++ -O3 -o timeSaverSolver -DTSS_WITH_EXPORT -DTSS_WITH_FULL_EXPORT main.cpp

.PHONY: update clean
clean:
	rm timeSaverSolver
update:
	git pull
