try {
./x64/Release/timeSaverSolver --cars 2 --outfile precomputed_tss --layout 0
./x64/Release/timeSaverSolver --cars 3 --outfile precomputed_tss --layout 0
./x64/Release/timeSaverSolver --cars 4 --outfile precomputed_tss --layout 0
./x64/Release/timeSaverSolver --cars 2 --outfile precomputed_tss --layout 1
./x64/Release/timeSaverSolver --cars 3 --outfile precomputed_tss --layout 1
./x64/Release/timeSaverSolver --cars 4 --outfile precomputed_tss --layout 1

mkdir -p web-build
C:\Users\daniel\AppData\Local\emsdk\upstream\emscripten\em++.bat jsTSS/web.cpp -c -g0 -O3 -DEMSCRIPTEN=1 -MD -o web-build/web.o
C:\Users\daniel\AppData\Local\emsdk\upstream\emscripten\em++.bat ./precomputed/precomputed_tss_classic_2.cpp -c -g0 -O3 -DEMSCRIPTEN=1 -MD -o web-build/classic_2.o
C:\Users\daniel\AppData\Local\emsdk\upstream\emscripten\em++.bat ./precomputed/precomputed_tss_classic_3.cpp -c -g0 -O3 -DEMSCRIPTEN=1 -MD -o web-build/classic_3.o
C:\Users\daniel\AppData\Local\emsdk\upstream\emscripten\em++.bat ./precomputed/precomputed_tss_classic_4.cpp -c -g0 -O3 -DEMSCRIPTEN=1 -MD -o web-build/classic_4.o
C:\Users\daniel\AppData\Local\emsdk\upstream\emscripten\em++.bat ./precomputed/precomputed_tss_inglenook_2.cpp -c -g0 -O3 -DEMSCRIPTEN=1 -MD -o web-build/inglenook_2.o
C:\Users\daniel\AppData\Local\emsdk\upstream\emscripten\em++.bat ./precomputed/precomputed_tss_inglenook_3.cpp -c -g0 -O3 -DEMSCRIPTEN=1 -MD -o web-build/inglenook_3.o
C:\Users\daniel\AppData\Local\emsdk\upstream\emscripten\em++.bat ./precomputed/precomputed_tss_inglenook_4.cpp -c -g0 -O3 -DEMSCRIPTEN=1 -MD -o web-build/inglenook_4.o
C:\Users\daniel\AppData\Local\emsdk\upstream\emscripten\em++.bat web-build/classic_2.o web-build/classic_3.o web-build/classic_4.o web-build/inglenook_2.o web-build/inglenook_3.o web-build/inglenook_4.o --bind -s EXPORTED_RUNTIME_METHODS=addFunction,ccall,UTF8ToString -s ALLOW_TABLE_GROWTH -O3 -o ./Emscripten/Release/jsTSS.html --shell-file jsTSS/shell.html
}
catch {
	exit 1
}