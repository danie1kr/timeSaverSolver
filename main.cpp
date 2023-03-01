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

#define CONNECTION(id, ...)  TimeSaver::Connection({__VA_ARGS__}, id)
#define FORWARD()  TimeSaver::DriveCondition(TimeSaver::DriveCondition::Direction::Forward)
#define BACKWARD()  TimeSaver::DriveCondition(TimeSaver::DriveCondition::Direction::Backward)
#define TURNOUT_AB(id)  TimeSaver::TurnoutCondition(id, TimeSaver::TurnoutCondition::Direction::A_B)
#define TURNOUT_AC(id)  TimeSaver::TurnoutCondition(id, TimeSaver::TurnoutCondition::Direction::A_C)


#define FWD(id)   CONNECTION(id, FORWARD())
#define BWD(id)   CONNECTION(id, BACKWARD())

#define FWD_AB  

    TimeSaver::Nodes<20> classic{{
        {0, {CONNECTION(1, FORWARD())}},
        {1, {CONNECTION(0, BACKWARD()), CONNECTION(2, FORWARD())}},
        {2, {CONNECTION(1, BACKWARD()), CONNECTION(3, FORWARD())}},
        {3, {CONNECTION(2, BACKWARD(), TURNOUT_AB(3)), CONNECTION(7, BACKWARD(), TURNOUT_AC(3)), CONNECTION(4, FORWARD())}},
        {4, {CONNECTION(3, BACKWARD()), CONNECTION(5, FORWARD())}},
        {5, {CONNECTION(4, BACKWARD()), CONNECTION(6, FORWARD())}},
        {6, {CONNECTION(5, BACKWARD())}},
        {7, {CONNECTION(11, BACKWARD()), CONNECTION(3, FORWARD(), TURNOUT_AB(7)), CONNECTION(8, FORWARD(), TURNOUT_AC(7))}},
        {8, {CONNECTION(7, BACKWARD()), CONNECTION(15, FORWARD())}},
        {9, {CONNECTION(10, FORWARD())}},
        {10, {CONNECTION(9, BACKWARD()), CONNECTION(11, FORWARD())}},
        {11, {CONNECTION(10, BACKWARD()), CONNECTION(12, FORWARD(), TURNOUT_AB(11)), CONNECTION(7, FORWARD(), TURNOUT_AC(11))}},
        {12, {CONNECTION(11, BACKWARD(), TURNOUT_AB(12)), CONNECTION(19, BACKWARD(), TURNOUT_AC(12)), CONNECTION(13, FORWARD())}},
        {13, {CONNECTION(12, BACKWARD()), CONNECTION(14, FORWARD())}},
        {14, {CONNECTION(13, BACKWARD()), CONNECTION(15, FORWARD())}},
        {15, {CONNECTION(14, BACKWARD(), TURNOUT_AB(15)), CONNECTION(8, BACKWARD(), TURNOUT_AC(15)), CONNECTION(16, FORWARD())}},
        {16, {CONNECTION(15, BACKWARD()), CONNECTION(17, FORWARD())}},
        {17, {CONNECTION(16, BACKWARD())}},
        {18, {CONNECTION(19, FORWARD())}},
        {19, {CONNECTION(18, BACKWARD()), CONNECTION(12, FORWARD())}},
    }};

    using TSS = TimeSaver::Solver<classic.size(), 1>;

#define S(i) " " #i ":" << std::setw(2) << state.slots[i] << std::setw(0)
#define T(i) "T" #i "[" << (state.turnouts[i] == TimeSaver::TurnoutCondition::Direction::A_B ? "A_B" : "A_C") <<"]:" << std::setw(2) << state.slots[i] << std::setw(0)

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