﻿#include <stdio.h>
#include <vector>
#include <string>
#include <map>
#include <emscripten.h>
#include <emscripten/bind.h>
#include <emscripten/val.h>

/*
* 
// DEBUG
em++ jsTSS/web.cpp -o Emscripten/Debug/web.o -D_DEBUG -g0 -O0 -DEMSCRIPTEN=1 -D__EMSCRIPTEN__=1 -MD -c
em++ Emscripten/Debug/web.o --bind -g3 -s EXPORTED_RUNTIME_METHODS=addFunction,ccall,UTF8ToString -s ALLOW_TABLE_GROWTH -O0 -o Emscripten/Debug/jsTSS.html --shell-file shell.html

// RELEASE
em++ jsTSS/web.cpp -o Emscripten/Release/web.o -g0 -O3 -DEMSCRIPTEN=1 -D__EMSCRIPTEN__=1 -MD -c
em++ Emscripten/Release/web.o --bind -s EXPORTED_RUNTIME_METHODS=addFunction,ccall,UTF8ToString -s ALLOW_TABLE_GROWTH -O3 -o Emscripten/Release/jsTSS.html --shell-file shell.html

*/

#define TSS_FLEXIBLE
#include "../tss.hpp"

#ifdef TSS_WITH_PACKED
#if __has_include("../precomputed/precomputed_tss_classic_2.hpp")
#include "../precomputed/precomputed_tss_classic_2.hpp"
#endif
#if __has_include("../precomputed/precomputed_tss_classic_3.hpp")
#include "../precomputed/precomputed_tss_classic_3.hpp"
#endif
#if __has_include("../precomputed/precomputed_tss_classic_4.hpp")
#include "../precomputed/precomputed_tss_classic_4.hpp"
#endif
#if __has_include("../precomputed/precomputed_tss_classic_5.hpp")
#include "../precomputed/precomputed_tss_classic_5.hpp"
#endif

#ifdef _DEBUG
const unsigned int tss_steps_classic_5_size = 0;
const TimeSaver::Solver::Precomputed::Step tss_steps_classic_5[] = { {0} };
const unsigned int tss_steps_classic_5_actions_size = 0;
const TimeSaver::Solver::Precomputed::Action tss_steps_classic_5_actions[] = { {0} };
#else
#endif
#endif

void (*errorJS)(const char* err) = nullptr;
void (*printJS)(const char* info, const char* state, const char* turnouts) = nullptr;
void (*stepJS)(unsigned int step, unsigned int steps, unsigned int solutions) = nullptr;
void (*statisticsJS)(unsigned int steps, unsigned int solutions) = nullptr;

void error(const unsigned int line, const std::string err)
{
	if (errorJS)
	{
		std::string text = std::to_string(line) + ": " + err;
		char* errJS = new char[text.size() + 1];
		memset(errJS, 0, text.size() + 1);
		text.copy(errJS, text.length());
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

const std::vector<Layout> layouts{ 
	{ "Classic",
		{ {
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
		} },
		{ {
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
		} }
	},
	{ "Inglenook",
	{ {
		{ 0, { FWD(1) }},
		{ 1, {BWD(0), FWD(2)} },
		{ 2, {BWD(1), FWD(3)} },
		{ 3, {BWD(2), FWD(4)} },
		{ 4, {BWD(3), FWD_A_B(5), FWD_A_C(10)} },
		{ 5, {BWD(4), FWD(6)} },
		{ 6, {BWD(5), FWD(7)} },
		{ 7, {BWD(6), FWD(8)} },
		{ 8, {BWD(7), FWD(9)} },
		{ 9, {BWD(8)} },
		{ 10, {BWD(4), FWD_A_B(14), FWD_A_C(11)} },
		{ 11, {BWD(10), FWD(12)} },
		{ 12, {BWD(11), FWD(13)} },
		{ 13, {BWD(12)} },
		{ 14, {BWD(10), FWD(15)} },
		{ 15, {BWD(14), FWD(16)} },
		{ 16, {BWD(15)} },
	} },
	{ {
		{10, 0},
		{20, 0},
		{30, 0},
		{40, 0},
		{50, 0},
		{60, 0},
		{70, 0},
		{80, 0},
		{90, 0},
		{100, 0},
		{60, 10},
		{70, 10},
		{80, 10},
		{90, 10},
		{70, 20},
		{80, 20},
		{90, 20}
	} }
	}
};

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

#ifdef TSS_WITH_PACKED
TimeSaver::Solver::Precomputed::Storage precomputedStepsGraph(unsigned int layout, unsigned int cars)
{
	if (layout == 0 && cars == 2)
		return { tss_steps_classic_2, tss_steps_classic_2_size, tss_steps_classic_2_actions, tss_steps_classic_2_actions_size };
	else if (layout == 0 && cars == 3)
		return { tss_steps_classic_3, tss_steps_classic_3_size, tss_steps_classic_3_actions, tss_steps_classic_3_actions_size };

#ifdef HAS_TSS_CLASSIC_4
	else if (layout == 0 && cars == 4)
		return { tss_steps_classic_4, tss_steps_classic_4_size, tss_steps_classic_4_actions, tss_steps_classic_4_actions_size };
#endif
#ifndef _DEBUG
#ifdef HAS_TSS_CLASSIC_5
	else if (layout == 0 && cars == 5)
		return { tss_steps_classic_5, tss_steps_classic_5_size, tss_steps_classic_5_actions, tss_steps_classic_5_actions_size };
#endif
#endif
	return { nullptr, 0, nullptr, 0 };
}
#endif

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

			for (unsigned int j = 0; j < n.connections.size(); ++j)
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

emscripten::val getMaximumCarCount(unsigned int layout) {
	
	return emscripten::val(
#ifdef _DEBUG
		layout == 0 ? 3 : 3
#else
		layout == 0 ? 5 : 5
#endif
	);
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
#ifndef TSS_WITH_PACKED
	InitSolver,
	StepSolver,
#endif
	InitDijkstra,
	StepDikjstra,
	MarkEndDijkstra,
	ShortestPathDikjstra,
	Finalize,
	Error
};

TSSState state = TSSState::InitTSS;
TimeSaver::Solver* solver = nullptr;
TimeSaver::Solver::Dijk::Path shortestPath = {};
#ifdef TSS_WITH_PACKED
unsigned int selectedStartStep = 0;
unsigned int selectedEndStep = 0;
#else
TimeSaver::Solver::CarPlacement carplacementStart;
TimeSaver::Solver::CarPlacement carplacementEnd;
const unsigned int selectedStartStep = 0;
const unsigned int selectedEndStep = 1;
unsigned int numberOfCars = 0;
#endif

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

emscripten::val getState()
{
	return emscripten::val(state);
}

emscripten::val getShortestPath()
{
	return emscripten::val(join(shortestPath, ","));
}

#define STRINGIFY(x) #x
emscripten::val timeSaverSolverInit(unsigned int layout, unsigned int numberOfCars)
{
	if (check() == false || state == TSSState::Error && state != TSSState::InitTSS)
	{
		error(__LINE__, "timeSaverSolver: Incompatible states. Please reload");
		return emscripten::val(TSSState::Error);
	}
	if (layout >= layouts.size() || numberOfCars < 1 || numberOfCars > layouts[layout].nodes.size())
	{
		error(__LINE__, "getRandomCarLayout: Incompatible arguments");
		return emscripten::val("");
	}

#ifdef TSS_WITH_PACKED
	auto print = [](const std::string info, const TimeSaver::Solver::PackedState& state) {};
#else
	auto print = [](const std::string info, const TimeSaver::Solver::State& state) {
		if (printJS)
			printJS(info.c_str(), nullptr, nullptr);
	};
#endif
	auto step = [](const unsigned int step, const unsigned int steps, const unsigned int solutions) {
		if (stepJS)
			stepJS(step, steps, solutions);
	};
	auto statistics = [](const unsigned int steps, const unsigned int solutions) {};
	
	solver = new TimeSaver::Solver(layouts[layout].nodes, print, step, statistics, distStorage, precStorage);

#ifdef TSS_WITH_PACKED
	solver->init(precomputedStepsGraph(layout, numberOfCars));
#else
	::numberOfCars = numberOfCars;
	//solver->solve_init(fromString(carPlacement));
#endif
	shortestPath = {};

	state = TSSState::ChooseStartAndEnd;

	return emscripten::val(state);
}

emscripten::val getCarLayout(unsigned int step)
{
	if (check() == false || state == TSSState::Error
#ifdef TSS_WITH_PACKED 
		|| step >= solver->stepsCount()
#endif
		)
	{
		error(__LINE__, "timeSaverSolver: Incompatible states. Please reload");
		return emscripten::val(TSSState::Error);
	}

#ifdef TSS_WITH_PACKED
	auto cars = solver->fromPackedStepsGraph(step);
#else
	auto cars = step == 0 ? carplacementStart : carplacementEnd;
#endif

	return emscripten::val(join(cars, ","));
}

emscripten::val getRandomStartState()
{
	if (check() == false || state == TSSState::Error || state != TSSState::ChooseStartAndEnd)
	{
		error(__LINE__, "timeSaverSolver: Incompatible states. Please reload");
		return emscripten::val(TSSState::Error);
	}

#ifdef TSS_WITH_PACKED
	selectedStartStep = std::rand() % solver->stepsCount();
#else
	carplacementStart = solver->random(numberOfCars);
#endif
	return emscripten::val(selectedStartStep);
}

emscripten::val getRandomEndState(unsigned int selectedStartStep, unsigned int difficulty)
{
	if (check() == false || state == TSSState::Error || state != TSSState::ChooseStartAndEnd)
	{
		error(__LINE__, "timeSaverSolver: Incompatible states. Please reload");
		return emscripten::val(TSSState::Error);
	}

#ifdef TSS_WITH_PACKED
	selectedEndStep = solver->randomEndState(selectedStartStep, difficulty);
	if (selectedStartStep != selectedEndStep)
		state = TSSState::InitDijkstra;
#else
	carplacementEnd = solver->random(numberOfCars);
	state = TSSState::InitSolver;
#endif
	return emscripten::val(selectedEndStep);
}

emscripten::val getExpectedSteps()
{
	if (check() == false || state == TSSState::Error)
	{
		error(__LINE__, "timeSaverSolver: Incompatible states. Please reload");
		return emscripten::val(TSSState::Error);
	}

	if (state == TSSState::InitDijkstra)
		return emscripten::val(solver->solve_dijkstra_expectedIterations());
	else
		return emscripten::val(0);
}

emscripten::val timeSaverSolver()
{
	if (check() == false || state == TSSState::Error || state == TSSState::ChooseStartAndEnd)
	{
		error(__LINE__, "timeSaverSolver: Incompatible states. Please reload");
		return emscripten::val(TSSState::Error);
	}

	switch (state)
	{
	default:
		return emscripten::val(TSSState::Error);
#ifndef TSS_WITH_PACKED
	case TSSState::InitSolver:
	{
		solver->init(carplacementStart);
		state = TSSState::StepSolver;
		break;
	}
	case TSSState::StepSolver:
	{
		auto result = solver->solve_step();
		if(result == false)
			state = TSSState::InitDijkstra;
		break;
	}
#endif
	case TSSState::InitDijkstra:
	{	
		solver->solve_dijkstra_init(selectedStartStep);
		state = TSSState::StepDikjstra;
		break;
	}
	case TSSState::StepDikjstra:
	{
		auto result = solver->solve_dijkstra_step/*<25>*/();
		if (result == false)
			state = TSSState::MarkEndDijkstra;
		break;
	}
	case TSSState::MarkEndDijkstra:
	{
#ifdef TSS_WITH_PACKED
		solver->solve_dijkstra_markEndStepsLike(selectedEndStep);
#else
		auto endSteps = solver->solve_dijkstra_markEndStepsLike(carplacementEnd);
		error(__LINE__, std::to_string(endSteps));
#endif
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
#ifndef TSS_WITH_PACKED
		.value("InitSolver", TSSState::InitSolver)
		.value("StepSolver", TSSState::StepSolver)
#endif
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
	function("getMaximumCarCount", &getMaximumCarCount);
	function("getCarLayout", &getCarLayout);
	function("getRandomStartState", &getRandomStartState);
	function("getRandomEndState", &getRandomEndState);
	function("getExpectedSteps", &getExpectedSteps);
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
	//EM_ASM({ Module.wasmTable = wasmTable; });
}
/*

int main() {
	printf("Hello, world!\n");
	return 0;
}*/