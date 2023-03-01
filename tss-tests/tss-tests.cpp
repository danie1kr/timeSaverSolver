#include "pch.h"
#include "CppUnitTest.h"
#include <iostream>
#include <iomanip>
#include <string>

#include "../tss.hpp"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

#define CONNECTION(id, ...)  TimeSaver::Connection({__VA_ARGS__}, id)
#define FORWARD()  TimeSaver::DriveCondition(TimeSaver::DriveCondition::Direction::Forward)
#define BACKWARD()  TimeSaver::DriveCondition(TimeSaver::DriveCondition::Direction::Backward)
#define TURNOUT_AB(id)  TimeSaver::TurnoutCondition(id, TimeSaver::TurnoutCondition::Direction::A_B)
#define TURNOUT_AC(id)  TimeSaver::TurnoutCondition(id, TimeSaver::TurnoutCondition::Direction::A_C)

#define S(i) " " #i ":" << std::setw(2) << state.slots[i] << std::setw(0)
#define T(i) "T" #i "[" << (state.turnouts[i] == TimeSaver::TurnoutCondition::Direction::A_B ? "A_B" : "A_C") <<"]:" << std::setw(2) << state.slots[i] << std::setw(0)

namespace tsstests
{
    using TSS = TimeSaver::Solver<5, 1>;
    using TSS_1car = TimeSaver::Solver<5, 2>;
    using TSS_2car = TimeSaver::Solver<5, 3>;

	TEST_CLASS(straight)
	{
        TimeSaver::Nodes<5> oneWay{ {
                {0, {CONNECTION(1, FORWARD())}},
                {1, {CONNECTION(0, BACKWARD()), CONNECTION(2, FORWARD())}},
                {2, {CONNECTION(1, BACKWARD()), CONNECTION(3, FORWARD())}},
                {3, {CONNECTION(2, BACKWARD()), CONNECTION(4, FORWARD())}},
                {4, {CONNECTION(3, BACKWARD())}},
            } };

        template<typename State>
        static void print(const unsigned int id, const State& state)
        {
            std::stringstream s;
            s << std::setfill(' ') << "Step: " << id << "\n" <<
                S(0) << " == " << S(1) << " == " << S(2) << " == " << S(3) << " == " << S(4) << "\n";
            Logger::WriteMessage(s.str().c_str());
        }
    
    public:
        TEST_METHOD(LocoOnlyForward)
        {
            TSS tss(oneWay, print<TSS::State>);
            TSS::CarPlacement cars{ { 0 } };
            TSS::CarPlacement target{ { 1 } };
            tss.init(cars);
            auto steps = tss.solve(target);
            Assert::IsTrue(steps == 2);
        }

        TEST_METHOD(LocoBackwardForward)
        {
            TSS tss(oneWay, print<TSS::State>);
            TSS::CarPlacement cars{ { 4 } };
            TSS::CarPlacement target{ { 1 } };
            tss.init(cars);
            auto steps = tss.solve(target);
            Assert::IsTrue(steps == 4);
        }

        TEST_METHOD(LocoPush)
        {
            using TSS = TSS_1car;
            TSS tss(oneWay, print<TSS::State>);
            TSS::CarPlacement cars{ { 0, 1 } };
            TSS::CarPlacement target{ { 1, 2 } };
            tss.init(cars);
            auto steps = tss.solve(target);
            Assert::IsTrue(steps == 2);
        }

        TEST_METHOD(LocoPushAndBack)
        {
            using TSS = TSS_1car;
            TSS tss(oneWay, print<TSS::State>);
            TSS::CarPlacement cars{ { 1, 3 } };
            TSS::CarPlacement target{ { 2, 4 } };
            tss.init(cars);
            auto steps = tss.solve(target);
            Assert::IsTrue(steps == 4);
        }

        TEST_METHOD(LocoPull)
        {
            using TSS = TSS_1car;
            TSS tss(oneWay, print<TSS::State>);
            TSS::CarPlacement cars{ { 0, 3 } };
            TSS::CarPlacement target{ { 1, 2 } };
            tss.init(cars);
            auto steps = tss.solve(target);
            Assert::IsTrue(steps == 4);
        }

        TEST_METHOD(LocoPullPush)
        {
            using TSS = TSS_2car;
            TSS tss(oneWay, print<TSS::State>);
            TSS::CarPlacement cars{ { 2, 0, 4 } };
            TSS::CarPlacement target{ { 2, 1, 3 } };
            tss.init(cars);
            auto steps = tss.solve(target);
            Assert::IsTrue(steps == 5);
        }
	};

    TEST_CLASS(oneTurnout)
    {
        using TSS_oneTurnout = TimeSaver::Solver<6, 1>;
        using TSS_oneTurnout_1car = TimeSaver::Solver<6, 2>;
        using TSS_oneTurnout_2car = TimeSaver::Solver<6, 3>;

        /*
        0 = 1 = T2 = 3 = 4
                 \\
                   5
        */
        TimeSaver::Nodes<6> turnout{ {
                {0, {CONNECTION(1, FORWARD())}},
                {1, {CONNECTION(0, BACKWARD()), CONNECTION(2, FORWARD())}},
                {2, {CONNECTION(1, BACKWARD()), CONNECTION(3, FORWARD(), TURNOUT_AB(2)), CONNECTION(5, FORWARD(), TURNOUT_AC(2))}},
                {3, {CONNECTION(2, BACKWARD(), TURNOUT_AB(2)), CONNECTION(4, FORWARD())}},
                {4, {CONNECTION(3, BACKWARD())}},
                {5, {CONNECTION(2, BACKWARD(), TURNOUT_AC(2))}},
            } };
        template<typename State>
        static void print(const unsigned int id, const State& state)
        {
            std::stringstream s;
            s << std::setfill(' ') << "Step: " << id << "\n" <<
                S(0) << " == " << S(1) << " == " << T(2) << " == " << S(3) << " == " << S(4) << "\n" <<
                "                  \\\\ " << S(5) << "\n";
            Logger::WriteMessage(s.str().c_str());
        };

    public:
        TEST_METHOD(LocoOnlyForwardAB)
        {
            using TSS = TSS_oneTurnout;
            TSS tss(turnout, print<TSS::State>);
            TSS::CarPlacement cars{ { 0 } };
            TSS::CarPlacement target{ { 3 } };
            tss.init(cars);
            auto steps = tss.solve(target);
            Assert::IsTrue(steps == 4);
        }

        TEST_METHOD(LocoOnlyBackwardAB)
        {
            using TSS = TSS_oneTurnout;
            TSS tss(turnout, print<TSS::State>);
            TSS::CarPlacement cars{ { 3 } };
            TSS::CarPlacement target{ { 1 } };
            tss.init(cars);
            auto steps = tss.solve(target);
            Assert::IsTrue(steps == 3);
        }

        TEST_METHOD(LocoOnlyForwardAC)
        {
            using TSS = TSS_oneTurnout;
            TSS tss(turnout, print<TSS::State>);
            TSS::CarPlacement cars{ { 0 } };
            TSS::CarPlacement target{ { 4 } };
            tss.init(cars);
            auto steps = tss.solve(target);
            Assert::IsTrue(steps == 5);
        }

        TEST_METHOD(LocoBackwardForward)
        {
            using TSS = TSS_oneTurnout;
            TSS tss(turnout, print<TSS::State>);
            TSS::CarPlacement cars{ { 3 } };
            TSS::CarPlacement target{ { 5 } };
            tss.init(cars);
            auto steps = tss.solve(target);
            Assert::IsTrue(steps == 5);
        }
       
        TEST_METHOD(LocoPushAB)
        {
            using TSS = TSS_oneTurnout_1car;
            TSS tss(turnout, print<TSS::State>);
            TSS::CarPlacement cars{ { 0, 1 } };
            TSS::CarPlacement target{ { 2, 3 } };
            tss.init(cars);
            auto steps = tss.solve(target);
            Assert::IsTrue(steps == 3);
        }

        TEST_METHOD(LocoPushAC)
        {
            using TSS = TSS_oneTurnout_1car;
            TSS tss(turnout, print<TSS::State>);
            TSS::CarPlacement cars{ { 0, 1 } };
            TSS::CarPlacement target{ { 1, 5 } };
            tss.init(cars);
            auto steps = tss.solve(target);
            Assert::IsTrue(steps == 4);
        }

        TEST_METHOD(LocoPullAB)
        {
            using TSS = TSS_oneTurnout_1car;
            TSS tss(turnout, print<TSS::State>);
            TSS::CarPlacement cars{ { 0, 3 } };
            TSS::CarPlacement target{ { 0, 1 } };
            tss.init(cars);
            auto steps = tss.solve(target);
            Assert::IsTrue(steps == 5);
        }

        TEST_METHOD(LocoPullAC)
        {
            using TSS = TSS_oneTurnout_1car;
            TSS tss(turnout, print<TSS::State>);
            TSS::CarPlacement cars{ { 3, 5 } };
            TSS::CarPlacement target{ { 0, 1 } };
            tss.init(cars);
            auto steps = tss.solve(target);
            Assert::IsTrue(steps == 6);
        }

        TEST_METHOD(CarReorder)
        {
            using TSS = TSS_oneTurnout_2car;
            TSS tss(turnout, print<TSS::State>);
            TSS::CarPlacement cars{ { 1, 2, 3 } };
            TSS::CarPlacement target{ { 1, 3, 2 } };
            tss.init(cars);
            auto steps = tss.solve(target);
            // not possible
            Assert::IsTrue(steps == 0);
        }
    };
    
    TEST_CLASS(oneTurnoutLongTracks)
    {
        using TSS_LongTracks_2car = TimeSaver::Solver<9, 3>;
        /*
        0 = 1 = 2 = T3 = 4 = 5 = 6
                     \\
                      7 = 8
        */
        TimeSaver::Nodes<9> turnout{ {
                {0, {CONNECTION(1, FORWARD())}},
                {1, {CONNECTION(0, BACKWARD()), CONNECTION(2, FORWARD())}},
                {2, {CONNECTION(1, BACKWARD()), CONNECTION(3, FORWARD())}},
                {3, {CONNECTION(2, BACKWARD()), CONNECTION(4, FORWARD(), TURNOUT_AB(3)), CONNECTION(7, FORWARD(), TURNOUT_AC(3))}},
                {4, {CONNECTION(3, BACKWARD(), TURNOUT_AB(3)), CONNECTION(5, FORWARD())}},
                {5, {CONNECTION(4, BACKWARD()), CONNECTION(6, FORWARD())}},
                {6, {CONNECTION(5, BACKWARD())}},
                {7, {CONNECTION(3, BACKWARD(), TURNOUT_AC(3)), CONNECTION(8, FORWARD())}},
                {8, {CONNECTION(7, BACKWARD())}}
            } };
        template<typename State>
        static void print(const unsigned int id, const State& state)
        {
            std::stringstream s;
            s << std::setfill(' ') << "Step: " << id << "\n" <<
                S(0) << " == " << S(1) << " == " << S(2) << " == " << T(3) << " == " << S(4) << " == " << S(5) << " == " << S(6) << "\n" <<
                "                                \\\\ " << S(7) << " == " << S(8) << "\n";
            Logger::WriteMessage(s.str().c_str());
        };

    public:
        TEST_METHOD(LocoPull)
        {
            using TSS = TSS_LongTracks_2car;
            TSS tss(turnout, print<TSS::State>);
            TSS::CarPlacement cars{ { 1, 3, 4 } };
            TSS::CarPlacement target{ { 1, 2, 3 } };
            tss.init(cars);
            auto steps = tss.solve(target);
            Assert::IsTrue(steps == 3);
        }

        TEST_METHOD(LocoReorder)
        {
            using TSS = TSS_LongTracks_2car;
            TSS tss(turnout, print<TSS::State>);
            TSS::CarPlacement cars{ { 1, 3, 4 } };
            TSS::CarPlacement target{ { 0, 4, 3 } };
            tss.init(cars);
            auto steps = tss.solve(target);
            Assert::IsTrue(steps == 14);
        }
    };
}
