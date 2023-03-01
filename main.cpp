#include <iostream>
#include <iomanip>

#include "tss.hpp"

int main()
{
    /*
        0=1=2=T3=4=5=6
             //
            T7 =  8  =
           //         \\
    9=10=T11=T12=13=14=T15=16=17
            //
        18=19

    T3 ?= 15
        */

#define FWD(id, ...)   TimeSaver::Connection(id, TimeSaver::Connection::Direction::Forward, __VA_ARGS__)
#define BWD(id, ...)   TimeSaver::Connection(id, TimeSaver::Connection::Direction::Backward, __VA_ARGS__)

#define A_B  TimeSaver::Connection::TurnoutState::A_B
#define A_C  TimeSaver::Connection::TurnoutState::A_C  

    TimeSaver::Nodes<20> classic{{
        {0, {FWD(1)}},
        {1, {BWD(0), FWD(2)}},
        {2, {BWD(1), FWD(3)}},
        {3, {BWD(2, A_B), BWD(7, A_C), FWD(4)}},
        {4, {BWD(3), FWD(5)}},
        {5, {BWD(4), FWD(6)}},
        {6, {BWD(5)}},
        {7, {BWD(11), FWD(3, A_B), FWD(8, A_C)}},
        {8, {BWD(7), FWD(15)}},
        {9, {FWD(10)}},
        {10, {BWD(9), FWD(11)}},
        {11, {BWD(10), FWD(12, A_B), FWD(7, A_C)}},
        {12, {BWD(11, A_B), BWD(19, A_C), FWD(13)}},
        {13, {BWD(12), FWD(14)}},
        {14, {BWD(13), FWD(15)}},
        {15, {BWD(14, A_B), BWD(8, A_C), FWD(16)}},
        {16, {BWD(15), FWD(17)}},
        {17, {BWD(16)}},
        {18, {FWD(19)}},
        {19, {BWD(18), FWD(12)}},
    }};

    using TSS = TimeSaver::Solver<classic.size(), 1>;

#define S(i) " " #i ":" << std::setw(2) << state.slots[i] << std::setw(0)
#define T(i) "T" #i "[" << (state.turnouts[i] == A_B ? "A_B" : "A_C") <<"]:" << std::setw(2) << state.slots[i] << std::setw(0)

    auto print = [](const unsigned int id, const TSS::State& state) {
        std::cout << std::setfill(' ') << "Step: " << id << "\n" <<
            S(0) << " == " << S(1) << " == " << S(2) << " == " << T(3) << " == " << S(4) << " == " << S(5) << " == " << S(6) << "\n" <<
            "                          //" << "\n" <<
            "                         " << T(7) << "" << " =============== " << S(8) << " ============== " << "\n" <<
            "                        //                                              \\\\\n" <<
            "    " << S(9) << " == " << S(10) << " == " << T(11) << " == " << T(12) << " == " << S(13) << " == " << S(14) << " == " << T(15) << " == " << S(16) << " == " << S(17) << "\n" <<
            "                       //\n" <<
            "       " << S(18) << " == " << S(19) << "\n\n";
    };

    TSS tss(classic, print);
    TSS::CarPlacement cars{ { 0 } };
    TSS::CarPlacement target{ { 14 } };
    tss.init(cars);
    tss.solve(target);
}