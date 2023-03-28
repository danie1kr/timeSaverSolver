#include <iostream>
#include <iomanip>
#include <iostream>
#include <fstream>

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

std::string varName(const std::string name, const size_t cars, const std::string suffix)
{
    return "tss_steps_" + name + "_" + std::to_string(cars) + suffix;
}

int main()
{
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


    TimeSaver::Nodes/*<20>*/ classic{{
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
    }};
    using TSS = TimeSaver::Solver;// <classic.size(), 5>;

#define S(i) " " #i ":" << std::setw(2) << state.slots[i] << std::setw(0)
#define T(i) "T" #i "[" << (state.turnouts[i] == TimeSaver::Connection::TurnoutState::A_B ? "A_B" : "A_C") <<"]:" << std::setw(2) << state.slots[i] << std::setw(0)
    auto printNone = [](const std::string info, const TSS::State& state) {};

    auto print = [](const std::string info, const TSS::State& state) {
        std::cout << std::setfill(' ') << info <<
            S(0) << " == " << S(1) << " == " << S(2) << " == " << T(3) << " == " << S(4) << " == " << S(5) << " == " << S(6) << "\n" <<
            "                          //" << "\n" <<
            "                         " << T(7) << "" << " ============ " << S(8) << " =========== " << "\n" <<
            "                        //                                              \\\\\n" <<
            "    " << S(9) << " == " << S(10) << " == " << T(11) << " == " << T(12) << " == " << S(13) << " == " << T(14) << " == " << S(15) << " == " << S(16) << "\n" <<
            "                                     //\n" <<
            "                     " << S(17) << " == " << S(18) << "\n\n";
    };
#undef A_B
#undef A_C

    auto stepNone = [](const unsigned int step, const unsigned int steps, const unsigned int solutions) {};
    auto step = [](const unsigned int step, const unsigned int steps, const unsigned int solutions) {
#ifndef _DEBUG
        if(step % 5000 == 0 || step == steps -1)
#endif
        std::cout << "\r" << "Step " << step << " / " << steps << " possible solutions: " << solutions;
    };

    auto statisticsNone = [](const unsigned int steps, const unsigned int solutions) {};
    auto statistics = [](const unsigned int steps, const unsigned int solutions)
    {
        std::cout << "\n" << "Step " << steps << " / " << steps << " possible solutions: " << solutions;
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
    
    TSS tss(classic, print, step, statistics, distStorage, precStorage);
//#define TSS_WITH_IMPORT
#ifdef TSS_WITH_IMPORT
#include "precomputed_tss.hpp"
    tss.init(tss_steps_for0);
    auto target = tss.randomFromStepsGraph();
    tss.solve(target);
#else
#ifdef TSS_WITH_EXPORT
    std::ofstream file("precomputed_tss.hpp", std::ofstream::out | std::ofstream::trunc);

#define STRINGIFY(x) #x
#define GENERATE(cars, layout, postfix) { \
        std::cout << "\n" << "Generating: " << STRINGIFY(layout) << " with " << cars << "\n"; \
        TSS tss(layout, printNone, step, statisticsNone, distStorage, precStorage); \
        tss.init(tss.random(cars), false); \
        tss.solve(tss.random(cars)); \
        tss.exportSteps(file, varName(STRINGIFY(layout), cars, postfix));  \
    }

    GENERATE(2, classic, "");
    GENERATE(3, classic, "");
    GENERATE(4, classic, "_a");
    GENERATE(4, classic, "_b");
    GENERATE(4, classic, "_c");
    file.close();
#endif
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