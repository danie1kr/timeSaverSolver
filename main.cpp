#include <iostream>
#include <iomanip>

#include "tss.hpp"

int main()
{
    /*
      0=1=2=T1=3=4=5
           //
          T2 = 6 =
         //       \\
    7=8=T3=T4=9=10=T5=11=12
           //
       13=14

    T3 ?= 15
        */

#define CONNECTION(id, ...)  TimeSaver::Connection({__VA_ARGS__}, id)
#define FORWARD()  TimeSaver::DriveCondition(TimeSaver::DriveCondition::Direction::Forward)
#define BACKWARD()  TimeSaver::DriveCondition(TimeSaver::DriveCondition::Direction::Backward)
#define TURNOUT_AB(id)  TimeSaver::TurnoutCondition(id, TimeSaver::TurnoutCondition::Direction::A_B)
#define TURNOUT_AC(id)  TimeSaver::TurnoutCondition(id, TimeSaver::TurnoutCondition::Direction::A_C)
    TimeSaver::Nodes<15> classic{{
        {0, {CONNECTION(1, FORWARD())}},
        {1, {CONNECTION(0, BACKWARD()), CONNECTION(2, FORWARD())}},
        {2, {CONNECTION(1, BACKWARD()), CONNECTION(3, FORWARD(), TURNOUT_AB(1))}},
        {3, {CONNECTION(2, BACKWARD(), TURNOUT_AB(1)), CONNECTION(8, BACKWARD(), TURNOUT_AC(1), TURNOUT_AB(2), TURNOUT_AC(3)), CONNECTION(4, FORWARD())}},
        {4, {CONNECTION(3, BACKWARD()), CONNECTION(5, FORWARD())}},
        {5, {CONNECTION(4, BACKWARD())}},
        {6, {CONNECTION(8, BACKWARD(), TURNOUT_AC(2), TURNOUT_AC(3)), CONNECTION(11, FORWARD(), TURNOUT_AC(5))}},
        {7, {CONNECTION(8, FORWARD())}},
        {8, {CONNECTION(7, BACKWARD()), CONNECTION(3, FORWARD(), TURNOUT_AC(1), TURNOUT_AB(2), TURNOUT_AC(3)), CONNECTION(6, FORWARD(), TURNOUT_AC(2), TURNOUT_AC(3)), CONNECTION(9, FORWARD(), TURNOUT_AB(3), TURNOUT_AB(4))}},
        {9, {CONNECTION(8, BACKWARD(), TURNOUT_AB(3), TURNOUT_AB(4)), CONNECTION(14, BACKWARD(), TURNOUT_AC(4)), CONNECTION(10, FORWARD())}},
        {10, {CONNECTION(9, BACKWARD()), CONNECTION(11, FORWARD(), TURNOUT_AB(5))}},
        {11, {CONNECTION(10, BACKWARD(), TURNOUT_AB(5)), CONNECTION(12, FORWARD())}},
        {12, {CONNECTION(11, BACKWARD())}},
        {13, {CONNECTION(14, FORWARD())}},
        {14, {CONNECTION(13, BACKWARD()), CONNECTION(9, FORWARD(), TURNOUT_AC(4))}},
    }};

    using TSS = TimeSaver::Solver<classic.size(), 1>;

#define S(i) "" #i ":" << std::setw(2) << state.slots[i] << std::setw(0)

    auto print = [](const unsigned int id, const TSS::State& state) {
        std::cout << std::setfill(' ') << "Step: " << id << "\n" <<
            S(0) << " == " << S(1) << " == " << S(2) << " == T1 == " << S(3) << " == " << S(4) << " == " << S(5) << "\n" <<
            "                       //" << "\n" <<
            "                      T2" << " ========= " << S(6) << " ======== " << "\n" <<
            "                     //                         \\\\\n" <<
            "    " << S(7) << " == " << S(8) << " == T3 == T4 == " << S(9) << " == " << S(10) << " == T5 == " << S(1) << " == " << S(12) << "\n" <<
            "                         //\n" <<
            "           " << S(13) << " == " << S(14) << "\n\n";
    };

    TSS tss(classic, print);
    TSS::CarPlacement cars{ { 0 } };
    TSS::CarPlacement target{ { 6 } };
    tss.init(cars);
    tss.solve(target);
}