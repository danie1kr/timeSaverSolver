#include <iostream>
#include <iomanip>

#define TSS_FLEXIBLE
#include "tss.hpp"

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

#define FWD(id, ...)   TimeSaver::Connection(id, TimeSaver::Connection::Direction::Forward, __VA_ARGS__)
#define BWD(id, ...)   TimeSaver::Connection(id, TimeSaver::Connection::Direction::Backward, __VA_ARGS__)

#define A_B  TimeSaver::Connection::TurnoutState::A_B
#define A_C  TimeSaver::Connection::TurnoutState::A_C  

    TimeSaver::Nodes/*<20>*/ classic{{
        {0, {FWD(1)}},
        {1, {BWD(0), FWD(2)}},
        {2, {BWD(1), FWD(3)}},
        {3, {BWD(2, A_B), BWD(7, A_C), FWD(4)}},
        {4, {BWD(3), FWD(5)}},
        {5, {BWD(4), FWD(6)}},
        {6, {BWD(5)}},
        {7, {BWD(11), FWD(3, A_B), FWD(8, A_C)}},
        {8, {BWD(7), FWD(14)}},
        {9, {FWD(10)}},
        {10, {BWD(9), FWD(11)}},
        {11, {BWD(10), FWD(12, A_B), FWD(7, A_C)}},
        {12 | (unsigned char)TimeSaver::Node::Options::CanParkCars, {BWD(11, A_B), BWD(18, A_C), FWD(13)}},
        {13, {BWD(12), FWD(14)}},
        {14, {BWD(13, A_B), BWD(8, A_C), FWD(15)}},
        {15, {BWD(14), FWD(16)}},
        {16, {BWD(15)}},
        {17, {FWD(18)}},
        {18, {BWD(17), FWD(12)}},
    }};

    using TSS = TimeSaver::Solver;// <classic.size(), 5>;

#define S(i) " " #i ":" << std::setw(2) << state.slots[i] << std::setw(0)
#define T(i) "T" #i "[" << (state.turnouts[i] == A_B ? "A_B" : "A_C") <<"]:" << std::setw(2) << state.slots[i] << std::setw(0)

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

    auto step = [](const unsigned int step, const unsigned int steps, const unsigned int solutions) {
#ifndef _DEBUG
        if(step % 5000 == 0 || step == steps -1)
#endif
        std::cout << "\r" << "Step " << step << " / " << steps << " possible solutions: " << solutions;
    };

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