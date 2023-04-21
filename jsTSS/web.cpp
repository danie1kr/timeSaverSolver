#include <stdio.h>
#include <vector>
#include <string>
#include <map>
#include <emscripten.h>
#include <emscripten/bind.h>
#include <emscripten/val.h>

#define TSS_FLEXIBLE
#include "../tss.hpp"

//#include "../precomputed_tss.hpp"

void (*errorJS)(const char* err) = nullptr;
void (*printJS)(const char* info, const char* state, const char* turnouts) = nullptr;
void (*stepJS)(unsigned int step, unsigned int steps, unsigned int solutions) = nullptr;
void (*statisticsJS)(unsigned int steps, unsigned int solutions) = nullptr;

void error(const std::string err)
{
	if (errorJS)
	{
		char* errJS = new char[err.size()+1];
		memset(errJS, 0, err.size() + 1);
		err.copy(errJS, err.length());
		errorJS(errJS);
		free(errJS);
	}
}

template<typename T>
const std::string join(T vector, const std::string delimiter = ",")
{

	std::string retVal = "";
	for (unsigned int i = 0; i < vector.size(); ++i)
		retVal += std::to_string(vector[i]) + (i == vector.size() - 1 ? "" : delimiter);

	return retVal;
}
// https://stackoverflow.com/questions/5167625/splitting-a-c-stdstring-using-tokens-e-g
std::vector<std::string> split(const std::string& s, char seperator)
{
	std::vector<std::string> output;
	std::string::size_type prev_pos = 0, pos = 0;

	while ((pos = s.find(seperator, pos)) != std::string::npos)
	{
		std::string substring(s.substr(prev_pos, pos - prev_pos));
		output.push_back(substring);
		prev_pos = ++pos;
	}

	output.push_back(s.substr(prev_pos, pos - prev_pos));
	return output;
}

TimeSaver::Solver::CarPlacement fromString(const std::string string)
{
	TimeSaver::Solver::CarPlacement carPlacement;

	auto strings = split(string, ',');
	carPlacement.resize(strings.size());
	for (auto& c : carPlacement)
		c = 0;

	for (unsigned int i = 0; i < strings.size(); ++i)
		carPlacement[i] = std::stoi(strings[i]);

	return carPlacement;
}


#define FWD(id)   TimeSaver::Connection(id, TimeSaver::Connection::Direction::Forward)
#define BWD(id)   TimeSaver::Connection(id, TimeSaver::Connection::Direction::Backward)
#define FWD_A_B(id)   TimeSaver::Connection(id, TimeSaver::Connection::Direction::Forward, TimeSaver::Connection::TurnoutState::A_B )
#define BWD_A_B(id)   TimeSaver::Connection(id, TimeSaver::Connection::Direction::Backward, TimeSaver::Connection::TurnoutState::A_B)
#define FWD_A_C(id)   TimeSaver::Connection(id, TimeSaver::Connection::Direction::Forward, TimeSaver::Connection::TurnoutState::A_C)
#define BWD_A_C(id)   TimeSaver::Connection(id, TimeSaver::Connection::Direction::Backward, TimeSaver::Connection::TurnoutState::A_C)

struct Position
{
	const unsigned int x, y;
};
using Positions = std::vector<Position>;

struct Layout
{
	std::string name;
	TimeSaver::Nodes nodes;
	Positions pos;
};

const std::vector<Layout> layouts{ { "Classic",
	{{
		{0, {FWD(1)}},
		{1, {BWD(0), FWD(2)}},
		{2, {BWD(1), FWD(3)}},
		{3, {BWD_A_B(2), BWD_A_C(7), FWD(4)}},
		{4, {BWD(3), FWD(5)}},
		{5, {BWD(4), FWD(6)}},
		{6, {BWD(5)}},
		{7, {BWD(11), FWD_A_B(3), FWD_A_C(8)}},
		{8, {BWD(7), FWD(14)}},
		{9, {FWD(10)}},
		{10, {BWD(9), FWD(11)}},
		{11, {BWD(10), FWD_A_B(12), FWD_A_C(7)}},
		{12 | (unsigned char)TimeSaver::Node::Options::CanParkCars, {BWD_A_B(11), BWD_A_C(18), FWD(13)}},
		{13, {BWD(12), FWD(14)}},
		{14, {BWD_A_B(13), BWD_A_C(8), FWD(15)}},
		{15, {BWD(14), FWD(16)}},
		{16, {BWD(15)}},
		{17, {FWD(18)}},
		{18, {BWD(17), FWD(12)}},
	}},
	{{
		{10, 0},
		{20, 0},
		{30, 0},
		{40, 0},
		{50, 0},
		{60, 0},
		{70, 0},
		{30, 10},
		{40, 10},
		{0, 20},
		{10, 20},
		{20, 20},
		{30, 20},
		{40, 20},
		{50, 20},
		{60, 20},
		{70, 20},
		{10, 30},
		{20, 30}
	}}
} };

std::vector<TimeSaver::Solver::DistanceStorage::StorageType> dist;
std::vector<TimeSaver::Solver::PrecStorage::StorageType> prec;
TimeSaver::Solver::DistanceStorage distStorage(
	[](const size_t elements, const size_t sizePerElement)
	{
		dist.resize(elements);
	},
	[](const size_t i, const TimeSaver::Solver::DistanceStorage::StorageType value)
	{
		dist[i] = value;
	},
		[](const size_t i) -> const TimeSaver::Solver::DistanceStorage::StorageType
	{
		return dist[i];
	}
	);

TimeSaver::Solver::PrecStorage precStorage(
	[](const size_t elements, const size_t sizePerElement)
	{
		prec.resize(elements);
	},
	[](const size_t i, const TimeSaver::Solver::PrecStorage::StorageType value)
	{
		prec[i] = value;
	},
		[](const size_t i) -> const TimeSaver::Solver::PrecStorage::StorageType
	{
		return prec[i];
	}
	);

TimeSaver::Solver::PackedSteps* precomputedStepsGraph(unsigned int layout, unsigned int cars)
{
	/*
	if (layout == 0 && cars == 2) 
		return &tss_steps_classic_2;
	
	error("cannot find precomputed steps graph");
	return &tss_steps_classic_2;*/
	return nullptr;
}

emscripten::val getLayoutCount() {
	return emscripten::val(layouts.size());
}
emscripten::val getLayoutName(unsigned int i) {
	if (i < layouts.size())
		return emscripten::val(layouts[i].name);
	else
		return emscripten::val("getLayoutName: incompatible index");
}

emscripten::val getLayoutNodes(unsigned int layout) {
	if (layout < layouts.size())
	{
		std::string nodes = "";

		for (unsigned int i = 0; i < layouts[layout].nodes.size(); ++i)
		{
			const auto n = layouts[layout].nodes[i];
			std::string node;

			node += "\"id\": " + std::to_string(n.id) + ", \"connections\": [";

			for (size_t j = 0; j < n.connections.size(); ++j)
			{
				node += "{ \"target\": " + std::to_string(n.connections[j].target)
					+ ", \"direction\": " + (n.connections[j].direction == TimeSaver::Connection::Direction::Forward ? "\"FWD\"" : "\"BWD\"")
					+ ", \"turnout\": " + (n.connections[j].turnoutState == TimeSaver::Connection::TurnoutState::None ? "\"None\"" : (n.connections[j].turnoutState == TimeSaver::Connection::TurnoutState::A_B ? "\"A_B\"" : "\"A_C\"")) + "}";
				if (j != n.connections.size() - 1)
					node += ", ";
			}
			node += "]";
			node += ", \"x\": " + std::to_string(layouts[layout].pos[i].x);
			node += ", \"y\": " + std::to_string(layouts[layout].pos[i].y);

			nodes += "{ " + node + " }";
			if (i != layouts[layout].nodes.size() - 1)
				nodes += ", ";
		}

		return emscripten::val("[" + nodes + "]");
	}
	else
		return emscripten::val("getLayoutName: incompatible index");
}

extern "C" {

	EMSCRIPTEN_KEEPALIVE
	void setCallbacks(
			void (*errorCallback)(const char* string),
			void (*printCallback)(const char* info, const char* state, const char* turnouts),
			void (*stepCallback)(unsigned int step, unsigned int steps, unsigned int solutions),
			void (*statisticsCallback)(unsigned int steps, unsigned int solutions)) {
		errorJS = errorCallback;
		printJS = printCallback;
		stepJS = stepCallback;
		statisticsJS = statisticsCallback;
	}
}

enum class TSSState : unsigned int
{
	InitTSS,
	ChooseStartAndEnd,
	InitDijkstra,
	StepDikjstra,
	MarkEndDijkstra,
	ShortestPathDikjstra,
	Finalize,
	Error
};

TSSState state = TSSState::InitTSS;
TimeSaver::Solver* solver = nullptr;
unsigned int shortestPath = 0;
unsigned int selectedStartStep = 0;
unsigned int selectedEndStep = 0;

bool check()
{
	if (state == TSSState::InitTSS)
	{
		if (solver)
			delete solver;
	}
	else
		if (solver == nullptr)
			return false;

	return true;
}

emscripten::val test()
{
	error("test!");
	return emscripten::val(0);
}

emscripten::val getState()
{
	return emscripten::val(state);
}

emscripten::val getShortestPath()
{
	return emscripten::val(shortestPath);
}

emscripten::val timeSaverSolverInit(unsigned int layout, unsigned int numberOfCars)
{
	if (check() == false || state == TSSState::Error && state != TSSState::InitTSS)
	{
		error("timeSaverSolver: Incompatible states. Please reload");
		return emscripten::val(TSSState::Error);
	}
	if (layout >= layouts.size() || numberOfCars < 1 || numberOfCars > layouts[layout].nodes.size())
	{
		error("getRandomCarLayout: Incompatible arguments");
		return emscripten::val("");
	}

	auto print = [](const std::string info, const TimeSaver::Solver::PackedState& state) {};
	auto step = [](const unsigned int step, const unsigned int steps, const unsigned int solutions) {
		if (stepJS)
			stepJS(step, steps, solutions);
	};
	auto statistics = [](const unsigned int steps, const unsigned int solutions) {};

	TimeSaver::Solver::DistanceStorage distStorage(
	[](const size_t elements, const size_t sizePerElement)
	{
		dist.resize(elements);
	},
	[](const size_t i, const TimeSaver::Solver::DistanceStorage::StorageType value)
	{
		dist[i] = value;
	},
		[](const size_t i) -> const TimeSaver::Solver::DistanceStorage::StorageType
	{
		return dist[i];
	}
	);

	TimeSaver::Solver::PrecStorage precStorage(
	[](const size_t elements, const size_t sizePerElement)
	{
		prec.resize(elements);
	},
	[](const size_t i, const TimeSaver::Solver::PrecStorage::StorageType value)
	{
		prec[i] = value;
	},
	[](const size_t i) -> const TimeSaver::Solver::PrecStorage::StorageType
	{
		return prec[i];
	}
	);

	solver = new TimeSaver::Solver(layouts[layout].nodes, print, step, statistics, distStorage, precStorage);
	solver->init(precomputedStepsGraph(layout, numberOfCars));
	//solver->solve_init(fromString(carPlacement));
	shortestPath = 0;
	state = TSSState::ChooseStartAndEnd;

	return emscripten::val(state);
}

emscripten::val getCarLayout(unsigned int step)
{
	if (check() == false || state == TSSState::Error || step >= solver->stepsCount())
	{
		error("timeSaverSolver: Incompatible states. Please reload");
		return emscripten::val(TSSState::Error);
	}

	auto cars = solver->fromPackedStepsGraph(step);

	return emscripten::val(join(cars, ","));
}

emscripten::val getRandomStartState()
{
	if (check() == false || state == TSSState::Error || state != TSSState::ChooseStartAndEnd)
	{
		error("timeSaverSolver: Incompatible states. Please reload");
		return emscripten::val(TSSState::Error);
	}

	selectedStartStep = std::rand() % solver->stepsCount();
	return emscripten::val(selectedStartStep);
}


emscripten::val getRandomEndState(unsigned int selectedStartStep, unsigned int difficulty)
{
	if (check() == false || state == TSSState::Error || state != TSSState::ChooseStartAndEnd)
	{
		error("timeSaverSolver: Incompatible states. Please reload");
		return emscripten::val(TSSState::Error);
	}

	selectedEndStep = solver->randomEndState(selectedStartStep, difficulty);
	return emscripten::val(selectedEndStep);
}

emscripten::val timeSaverSolver()
{
	if (check() == false || state == TSSState::Error || state == TSSState::InitTSS)
	{
		error("timeSaverSolver: Incompatible states. Please reload");
		return emscripten::val(TSSState::Error);
	}

	switch (state)
	{
	default:
		return emscripten::val(TSSState::Error);
	case TSSState::InitDijkstra:
	{	
		solver->solve_dijkstra_init(selectedStartStep);
		state = TSSState::StepDikjstra;
		break;
	}
	case TSSState::StepDikjstra:
	{
		auto result = solver->solve_dijkstra_step();
		if (result == false)
			state = TSSState::MarkEndDijkstra;
		break;
	}
	case TSSState::MarkEndDijkstra:
	{
		solver->solve_dijkstra_markEndStepsLike(selectedEndStep);
		state = TSSState::ShortestPathDikjstra;
		break;
	}
	case TSSState::ShortestPathDikjstra:
	{	
		shortestPath = solver->solve_dijkstra_shortestPath();
		state = TSSState::Finalize;
		break;
	}
	case TSSState::Finalize:
	{	
		break;
	}
	}

	return emscripten::val(state);
}


EMSCRIPTEN_BINDINGS(timeSaverSolver) {
	emscripten::enum_<TSSState>("TSSState")
		.value("InitTSS", TSSState::InitTSS)
		.value("ChooseStartAndEnd", TSSState::ChooseStartAndEnd)
		.value("InitDijkstra", TSSState::InitDijkstra)
		.value("StepDikjstra", TSSState::StepDikjstra)
		.value("MarkEndDijkstra", TSSState::MarkEndDijkstra)
		.value("ShortestPathDikjstra", TSSState::ShortestPathDikjstra)
		.value("Finalize", TSSState::Finalize)
		.value("Error", TSSState::Error)
		;

	function("getLayoutCount", &getLayoutCount);
	function("getLayoutName", &getLayoutName);
	function("getLayoutNodes", &getLayoutNodes);
	function("getCarLayout", &getCarLayout);
	function("getRandomStartState", &getRandomStartState);
	function("getRandomEndState", &getRandomEndState);
	function("test", &test);
	function("timeSaverSolverInit", &timeSaverSolverInit);
	function("timeSaverSolver", &timeSaverSolver);
	function("getState", &getState);
	function("getShortestPath", &getShortestPath);
	
	emscripten::enum_<TimeSaver::Connection::TurnoutState>("TurnoutState")
		.value("A_B", TimeSaver::Connection::TurnoutState::A_B)
		.value("A_C", TimeSaver::Connection::TurnoutState::A_C)
		.value("None", TimeSaver::Connection::TurnoutState::None)
		;


	emscripten::value_object<TimeSaver::Solver::State>("State")
		.field("slots", &TimeSaver::Solver::State::slots)
		.field("turnouts", &TimeSaver::Solver::State::turnouts)
		;
}
/*

int main() {
	printf("Hello, world!\n");
	return 0;
}*/