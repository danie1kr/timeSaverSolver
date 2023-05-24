#include <iostream>
#include <iomanip>
#include <iostream>
#include <fstream>

#include "external/argparse.hpp"

#define TSS_FLEXIBLE
#include "tss.hpp"

enum class Layout : unsigned char
{
    Classic = 0,
    Inglenook = 1
};

std::vector<std::vector< TimeSaver::Solver::Steps*>> allSteps;

const size_t carsIndex(const size_t cars)
{
    switch (cars)
    {
    default:
        return 0;
    case 1: return 1;
    case 2: return 1;
    case 3: return 3;
    case 4: return 4;
    case 5: return 5;
    }
}

const size_t stepsIndex(const Layout layout, const size_t cars)
{
    const auto perLayout = carsIndex(1) + carsIndex(2) + carsIndex(3) + carsIndex(4) + carsIndex(5);
    
    return (size_t)layout * perLayout + carsIndex(cars);
}

std::string varName(const std::string name, const size_t cars)
{
    return "tss_steps_" + name + "_" + std::to_string(cars);
}

#ifdef TSS_WITH_IMPORT
#define TSS_FLEXIBLE
#include "tss.hpp"
#include "precomputed/precomputed_tss_classic_2.hpp"
#include "precomputed/precomputed_tss_classic_3.hpp"
#include "precomputed/precomputed_tss_classic_4.hpp"
#include "precomputed/precomputed_tss_classic_5.hpp"

#ifdef _DEBUG
const unsigned int tss_steps_classic_4_size = 0;
const TimeSaver::Solver::Precomputed::Step tss_steps_classic_4[] = { {0} };
const unsigned int tss_steps_classic_4_actions_size = 0;
const TimeSaver::Solver::Precomputed::Action tss_steps_classic_4_actions[] = { {0} };
const unsigned int tss_steps_classic_5_size = 0;
const TimeSaver::Solver::Precomputed::Step tss_steps_classic_5[] = { {0} };
const unsigned int tss_steps_classic_5_actions_size = 0;
const TimeSaver::Solver::Precomputed::Action tss_steps_classic_5_actions[] = { {0} };
#endif

TimeSaver::Solver::Precomputed::Storage precomputedStepsGraph(unsigned int layout, unsigned int cars)
{
    if (layout == 0 && cars == 2)
        return { tss_steps_classic_2, tss_steps_classic_2_size, tss_steps_classic_2_actions, tss_steps_classic_2_actions_size };
    else if (layout == 0 && cars == 3)
        return { tss_steps_classic_3, tss_steps_classic_3_size, tss_steps_classic_3_actions, tss_steps_classic_3_actions_size };
#ifndef _DEBUG
    else if (layout == 0 && cars == 4)
        return { tss_steps_classic_4, tss_steps_classic_4_size, tss_steps_classic_4_actions, tss_steps_classic_4_actions_size };
    else if (layout == 0 && cars == 5)
        return { tss_steps_classic_5, tss_steps_classic_5_size, tss_steps_classic_5_actions, tss_steps_classic_5_actions_size };
#endif
    return { nullptr, 0, nullptr, 0 };
}
#endif

int main(int argc, const char* const argv[])
{
    argparse::ArgumentParser argparser("timeSaverSolver");

    argparser.add_argument("--cars")
        .required()
        .help("number of cars to compute")
        .metavar("CARS")
        .scan<'i', int>();

    argparser.add_argument("--outfile")
        .help("output file")
        .required()
        .metavar("OUTFILE");

    try {
        argparser.parse_args(argc, argv);
    }
    catch (const std::runtime_error& err) {
        std::cerr << err.what() << std::endl;
        std::cerr << argparser;
        std::exit(1);
    }

    /*
        0=1=2=T3=4=5=6
             //
            T7 = 8 =
           //       \\
    9=10=T11=T12=13=T14=15=16
            //
        17=18

    T3 ?= 15
        */

#define FWD(id)   TimeSaver::Connection(id, TimeSaver::Connection::Direction::Forward)
#define BWD(id)   TimeSaver::Connection(id, TimeSaver::Connection::Direction::Backward)
#define FWD_A_B(id)   TimeSaver::Connection(id, TimeSaver::Connection::Direction::Forward, TimeSaver::Connection::TurnoutState::A_B )
#define BWD_A_B(id)   TimeSaver::Connection(id, TimeSaver::Connection::Direction::Backward, TimeSaver::Connection::TurnoutState::A_B)
#define FWD_A_C(id)   TimeSaver::Connection(id, TimeSaver::Connection::Direction::Forward, TimeSaver::Connection::TurnoutState::A_C)
#define BWD_A_C(id)   TimeSaver::Connection(id, TimeSaver::Connection::Direction::Backward, TimeSaver::Connection::TurnoutState::A_C)

    TimeSaver::Nodes/*<20>*/ classic{ {
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
    } };
    using TSS = TimeSaver::Solver;// <classic.size(), 5>;


#define S(i) " " #i ":" << std::setw(2) << state.node(i) << std::setw(0)
#define T(i,t) "T" #i "[" << (state.turnoutState(t) == TimeSaver::Connection::TurnoutState::DontCare ? "_?_" : (state.turnoutState(t) == TimeSaver::Connection::TurnoutState::A_B ? "A_B" : "A_C")) <<"]:" << std::setw(2) << state.node(i) << std::setw(0)
    auto printNone = [](const std::string info, const TSS::PackedState& state) {};

    auto print = [](const std::string info, const TSS::PackedState& state) {
        std::cout << std::setfill(' ') << info <<
            S(0) << " == " << S(1) << " == " << S(2) << " == " << T(3, 0) << " == " << S(4) << " == " << S(5) << " == " << S(6) << "\n" <<
            "                          //" << "\n" <<
            "                         " << T(7, 1) << "" << " ============ " << S(8) << " =========== " << "\n" <<
            "                        //                                              \\\\\n" <<
            "    " << S(9) << " == " << S(10) << " == " << T(11, 2) << " == " << T(12, 3) << " == " << S(13) << " == " << T(14, 4) << " == " << S(15) << " == " << S(16) << "\n" <<
            "                                     //\n" <<
            "                     " << S(17) << " == " << S(18) << "\n\n";
    };
#undef A_B
#undef A_C

    auto stepNone = [](const unsigned int step, const unsigned int steps, const unsigned int solutions) {};
    auto step = [](const unsigned int step, const unsigned int steps, const unsigned int solutions) {
#ifndef _DEBUG
        if (step % 5000 == 0 || step == steps - 1)
#endif
            std::cout << "\r" << "Step " << step << " / " << steps << " possible solutions: " << solutions << std::flush;
    };

    auto statisticsNone = [](const unsigned int steps, const unsigned int solutions) {};
    auto statistics = [](const unsigned int steps, const unsigned int solutions)
    {
        std::cout << "\n" << "Step " << steps << " / " << steps << " possible solutions: " << solutions << std::flush;
    };

    std::vector<TSS::DistanceStorage::StorageType> dist;
    TSS::DistanceStorage distStorage(
        [&dist](const size_t elements, const size_t sizePerElement)
        {
            dist.resize(elements);
        },
        [&dist](const size_t i, const TSS::DistanceStorage::StorageType value)
        {
            dist[i] = value;
        },
            [&dist](const size_t i) -> const TSS::DistanceStorage::StorageType
        {
            return dist[i];
        }
        );

    std::vector<TSS::PrecStorage::StorageType> prec;
    TSS::PrecStorage precStorage(
        [&prec](const size_t elements, const size_t sizePerElement)
        {
            prec.resize(elements);
        },
        [&prec](const size_t i, const TSS::PrecStorage::StorageType value)
        {
            prec[i] = value;
        },
            [&prec](const size_t i) -> const TSS::PrecStorage::StorageType
        {
            return prec[i];
        }
        );

#define STRINGIFY(x) #x
#define FILENAME(prefix, cars, layout)  prefix + "_" + STRINGIFY(layout) + "_" + std::to_string(cars)
#define GENERATE(cars, layout, cpp, hpp, hppName) { \
        std::cout << "\n" << "Generating: " << STRINGIFY(layout) << " with " << cars << "\n" << std::flush; \
        TSS tss(layout, printNone, step, statisticsNone, distStorage, precStorage); \
        tss.init(tss.random(cars), false); \
        tss.createGraph(); \
        tss.exportSteps(cpp, hpp, hppName, varName(STRINGIFY(layout), cars));  \
    }

    //#define TSS_WITH_IMPORT
#ifdef TSS_WITH_IMPORT
#ifdef _DEBUG
    const unsigned int tss_steps_classic_4_size = 0;
    const TimeSaver::Solver::Precomputed::Step tss_steps_classic_4[] = { {0} };
    const unsigned int tss_steps_classic_4_actions_size = 0;
    const TimeSaver::Solver::Precomputed::Action tss_steps_classic_4_actions[] = { {0} };
    const unsigned int tss_steps_classic_5_size = 0;
    const TimeSaver::Solver::Precomputed::Step tss_steps_classic_5[] = { {0} };
    const unsigned int tss_steps_classic_5_actions_size = 0;
    const TimeSaver::Solver::Precomputed::Action tss_steps_classic_5_actions[] = { {0} };
#endif

    const auto layout = 0;
    const auto cars = 3;

    TimeSaver::Solver* solver = nullptr;
    solver = new TimeSaver::Solver(classic , print, step, statistics, distStorage, precStorage);
    solver->init(precomputedStepsGraph(layout, cars));
    auto randomStartStep = std::rand() % solver->stepsCount();
    auto carPlacementStart = solver->fromPackedStepsGraph(randomStartStep);
    auto difficulty = 0;
    auto randomEndStep = solver->randomEndState(randomStartStep, difficulty);
    auto carPlacementEnd = solver->fromPackedStepsGraph(randomEndStep);
    solver->solve_dijkstra_init(randomStartStep);

    bool result = false;
    do
    {
        result = solver->solve_dijkstra_step();
    } while (result);

    solver->solve_dijkstra_markEndStepsLike(randomEndStep);
    auto shortestPath = solver->solve_dijkstra_shortestPath();
#else
#ifdef TSS_WITH_EXPORT

    TSS tss(classic, print, step, statistics, distStorage, precStorage);
    if (auto outputFile = argparser.present("--outfile"))
    {
        auto cars = argparser.get<int>("--cars");
        std::ofstream hpp("precomputed/" + FILENAME(outputFile.value(), cars, classic) + ".hpp", std::ofstream::out | std::ofstream::trunc);
        std::ofstream cpp("precomputed/" + FILENAME(outputFile.value(), cars, classic) + ".cpp", std::ofstream::out | std::ofstream::trunc);
        GENERATE(cars, classic, cpp, hpp, FILENAME(outputFile.value(), cars, classic) + ".hpp");
        cpp.close();
        hpp.close();
    }

#endif // !_DEBUG
#endif

    /*
    for (unsigned int i = 0; i < 5; ++i)
    {
        TSS tss(classic, print, step, statistics, distStorage, precStorage);
        auto cars = tss.random(2);
        auto target = tss.random(2);
        tss.init(cars, false);
        tss.solve(target);
        std::cout << "it: " << i << " known steps in graph: " << tss.steps_count() << "\n";
    }

    /*
    //TSS::CarPlacement cars{ { 0 } };
    //TSS::CarPlacement target{ { 14 } };

    auto cars = tss.random(5);
    //TSS::CarPlacement target = { {10,9} };
    auto target = tss.random(5);
    //TSS::CarPlacement target = { {1,0} };
    tss.init(cars);
    tss.solve(target);
    */
}