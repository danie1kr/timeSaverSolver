try {
./x64/Release/timeSaverSolver --cars 2 --outfile precomputed_tss --layout 0
./x64/Release/timeSaverSolver --cars 3 --outfile precomputed_tss --layout 0
./x64/Release/timeSaverSolver --cars 4 --outfile precomputed_tss --layout 0
./x64/Release/timeSaverSolver --cars 2 --outfile precomputed_tss --layout 1
./x64/Release/timeSaverSolver --cars 3 --outfile precomputed_tss --layout 1
./x64/Release/timeSaverSolver --cars 4 --outfile precomputed_tss --layout 1
./x64/Release/timeSaverSolver --cars 5 --outfile precomputed_tss --layout 1
}
catch {
	exit 1
}