solver: main.cpp
	g++ -std=c++2a -O3 -o timeSaverSolver -DTSS_WITH_EXPORT main.cpp

PRECOMPUTED=$(wildcard precomputed/precomputed_tss*.cpp)
OBJECTS=$(patsubst %.cpp, %.o, $(PRECOMPUTED))

web: $(OBJECTS) web.o
	em++ $(OBJECTS) web.o --bind -s EXPORTED_RUNTIME_METHODS=addFunction,ccall,UTF8ToString -s ALLOW_TABLE_GROWTH -O3 -o Emscripten/Release/jsTSS.html --shell-file jsTSS/shell.html

web.o: jsTSS/web.cpp
	em++ $< -c -g0 -O3 -DEMSCRIPTEN=1 -D__EMSCRIPTEN__=1 -MD -o $@

$(OBJECTS): %.o : %.cpp
	em++ $< -c -g0 -O3 -DEMSCRIPTEN=1 -D__EMSCRIPTEN__=1 -MD -o $@


.PHONY: update clean
clean:
	rm timeSaverSolver *.o
update:
	git pull
