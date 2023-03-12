#include "pch.h"
#include "CppUnitTest.h"
#include <iostream>
#include <iomanip>
#include <string>

#include "../tss.hpp"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;


#define FWD(id, ...)   TimeSaver::Connection(id, TimeSaver::Connection::Direction::Forward, __VA_ARGS__)
#define BWD(id, ...)   TimeSaver::Connection(id, TimeSaver::Connection::Direction::Backward, __VA_ARGS__)

#define A_B  TimeSaver::Connection::TurnoutState::A_B
#define A_C  TimeSaver::Connection::TurnoutState::A_C  

#define S(i) " " #i ":" << std::setw(2) << state.slots[i] << std::setw(0)
#define T(i) "T" #i "[" << (state.turnouts[i] == A_B ? "A_B" : "A_C") <<"]:" << std::setw(2) << state.slots[i] << std::setw(0)

namespace tsstests
{
    using TSS = TimeSaver::Solver<5, 1>;
    using TSS_1car = TimeSaver::Solver<5, 2>;
    using TSS_2car = TimeSaver::Solver<5, 3>;

    auto step = [](const unsigned int step, const unsigned int steps, const unsigned int solutions) {

    };

    auto statistics = [](const unsigned int steps, const unsigned int solutions)
    {

    };

	TEST_CLASS(straight)
	{
        TimeSaver::Nodes<5> oneWay{ {
                {0, {FWD(1)}},
                {1, {BWD(0), FWD(2)}},
                {2, {BWD(1), FWD(3)}},
                {3, {BWD(2), FWD(4)}},
                {4, {BWD(3)}},
            } };

        template<typename State>
        static void print(const std::string info, const State& state)
        {
            std::stringstream s;
            s << std::setfill(' ') << info <<
                S(0) << " == " << S(1) << " == " << S(2) << " == " << S(3) << " == " << S(4) << "\n";
            Logger::WriteMessage(s.str().c_str());
        }
    
    public:
        TEST_METHOD(LocoOnlyForward)
        {
            TSS tss(oneWay, print<TSS::State>, step, statistics);
            TSS::CarPlacement cars{ { 0 } };
            TSS::CarPlacement target{ { 1 } };
            tss.init(cars);
            auto steps = tss.solve(target);
            Assert::IsTrue(steps == 2);
        }

        TEST_METHOD(LocoBackwardForward)
        {
            TSS tss(oneWay, print<TSS::State>, step, statistics);
            TSS::CarPlacement cars{ { 4 } };
            TSS::CarPlacement target{ { 1 } };
            tss.init(cars);
            auto steps = tss.solve(target);
            Assert::IsTrue(steps == 4);
        }

        TEST_METHOD(LocoPush)
        {
            using TSS = TSS_1car;
            TSS tss(oneWay, print<TSS::State>, step, statistics);
            TSS::CarPlacement cars{ { 0, 1 } };
            TSS::CarPlacement target{ { 1, 2 } };
            tss.init(cars);
            auto steps = tss.solve(target);
            Assert::IsTrue(steps == 2);
        }

        TEST_METHOD(LocoPushAndBack)
        {
            using TSS = TSS_1car;
            TSS tss(oneWay, print<TSS::State>, step, statistics);
            TSS::CarPlacement cars{ { 1, 3 } };
            TSS::CarPlacement target{ { 2, 4 } };
            tss.init(cars);
            auto steps = tss.solve(target);
            Assert::IsTrue(steps == 4);
        }

        TEST_METHOD(LocoPull)
        {
            using TSS = TSS_1car;
            TSS tss(oneWay, print<TSS::State>, step, statistics);
            TSS::CarPlacement cars{ { 0, 3 } };
            TSS::CarPlacement target{ { 1, 2 } };
            tss.init(cars);
            auto steps = tss.solve(target);
            Assert::IsTrue(steps == 4);
        }

        TEST_METHOD(LocoPullPush)
        {
            using TSS = TSS_2car;
            TSS tss(oneWay, print<TSS::State>, step, statistics);
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
                {0, {FWD(1)}},
                {1, {BWD(0), FWD(2)}},
                {2, {BWD(1), FWD(3, A_B), FWD(5, A_C)}},
                {3, {BWD(2), FWD(4)}},
                {4, {BWD(3)}},
                {5, {BWD(2)}},
            } };
        template<typename State>        
        static void print(const std::string info, const State& state)
        {
            std::stringstream s;
            s << std::setfill(' ') << info <<
                S(0) << " == " << S(1) << " == " << T(2) << " == " << S(3) << " == " << S(4) << "\n" <<
                "                  \\\\ " << S(5) << "\n";
            Logger::WriteMessage(s.str().c_str());
        };

    public:
        TEST_METHOD(LocoOnlyForwardAB)
        {
            using TSS = TSS_oneTurnout;
            TSS tss(turnout, print<TSS::State>, step, statistics);
            TSS::CarPlacement cars{ { 0 } };
            TSS::CarPlacement target{ { 3 } };
            tss.init(cars);
            auto steps = tss.solve(target);
            Assert::IsTrue(steps == 4);
        }

        TEST_METHOD(LocoOnlyBackwardAB)
        {
            using TSS = TSS_oneTurnout;
            TSS tss(turnout, print<TSS::State>, step, statistics);
            TSS::CarPlacement cars{ { 3 } };
            TSS::CarPlacement target{ { 1 } };
            tss.init(cars);
            auto steps = tss.solve(target);
            Assert::IsTrue(steps == 3);
        }

        TEST_METHOD(LocoOnlyForwardAC)
        {
            using TSS = TSS_oneTurnout;
            TSS tss(turnout, print<TSS::State>, step, statistics);
            TSS::CarPlacement cars{ { 0 } };
            TSS::CarPlacement target{ { 4 } };
            tss.init(cars);
            auto steps = tss.solve(target);
            Assert::IsTrue(steps == 5);
        }

        TEST_METHOD(LocoBackwardForward)
        {
            using TSS = TSS_oneTurnout;
            TSS tss(turnout, print<TSS::State>, step, statistics);
            TSS::CarPlacement cars{ { 3 } };
            TSS::CarPlacement target{ { 5 } };
            tss.init(cars);
            auto steps = tss.solve(target);
            Assert::IsTrue(steps == 5);
        }
       
        TEST_METHOD(LocoPushAB)
        {
            using TSS = TSS_oneTurnout_1car;
            TSS tss(turnout, print<TSS::State>, step, statistics);
            TSS::CarPlacement cars{ { 0, 1 } };
            TSS::CarPlacement target{ { 2, 3 } };
            tss.init(cars);
            auto steps = tss.solve(target);
            Assert::IsTrue(steps == 3);
        }

        TEST_METHOD(LocoPushAC)
        {
            using TSS = TSS_oneTurnout_1car;
            TSS tss(turnout, print<TSS::State>, step, statistics);
            TSS::CarPlacement cars{ { 0, 1 } };
            TSS::CarPlacement target{ { 1, 5 } };
            tss.init(cars);
            auto steps = tss.solve(target);
            Assert::IsTrue(steps == 4);
        }

        TEST_METHOD(LocoPullAB)
        {
            using TSS = TSS_oneTurnout_1car;
            TSS tss(turnout, print<TSS::State>, step, statistics);
            TSS::CarPlacement cars{ { 0, 3 } };
            TSS::CarPlacement target{ { 0, 1 } };
            tss.init(cars);
            auto steps = tss.solve(target);
            Assert::IsTrue(steps == 5);
        }

        TEST_METHOD(LocoPullAC)
        {
            using TSS = TSS_oneTurnout_1car;
            TSS tss(turnout, print<TSS::State>, step, statistics);
            TSS::CarPlacement cars{ { 3, 5 } };
            TSS::CarPlacement target{ { 0, 1 } };
            tss.init(cars);
            auto steps = tss.solve(target);
            Assert::IsTrue(steps == 6);
        }

        TEST_METHOD(CarReorder)
        {
            using TSS = TSS_oneTurnout_2car;
            TSS tss(turnout, print<TSS::State>, step, statistics);
            TSS::CarPlacement cars{ { 1, 2, 3 } };
            TSS::CarPlacement target{ { 1, 3, 2 } };
            tss.init(cars);
            auto steps = tss.solve(target, false);
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
                {0, {FWD(1)}},
                {1, {BWD(0), FWD(2)}},
                {2, {BWD(1), FWD(3)}},
                {3, {BWD(2), FWD(4, A_B), FWD(7, A_C)}},
                {4, {BWD(3), FWD(5)}},
                {5, {BWD(4), FWD(6)}},
                {6, {BWD(5)}},
                {7, {BWD(3), FWD(8)}},
                {8, {BWD(7)}}
            } };
        template<typename State>
        static void print(const std::string info, const State& state)
        {
            std::stringstream s;
            s << std::setfill(' ') << info <<
                S(0) << " == " << S(1) << " == " << S(2) << " == " << T(3) << " == " << S(4) << " == " << S(5) << " == " << S(6) << "\n" <<
                "                                \\\\ " << S(7) << " == " << S(8) << "\n";
            Logger::WriteMessage(s.str().c_str());
        };

    public:
        TEST_METHOD(LocoPull)
        {
            using TSS = TSS_LongTracks_2car;
            TSS tss(turnout, print<TSS::State>, step, statistics);
            TSS::CarPlacement cars{ { 1, 4, 5 } };
            TSS::CarPlacement target{ { 1, 2, 3 } };
            tss.init(cars);
            auto steps = tss.solve(target);
            Assert::IsTrue(steps == 5);
        }

        TEST_METHOD(LocoReorder)
        {
            using TSS = TSS_LongTracks_2car;
            TSS tss(turnout, print<TSS::State>, step, statistics);
            TSS::CarPlacement cars{ { 1, 3, 4 } };
            TSS::CarPlacement target{ { 0, 5, 4 } };
            tss.init(cars);
            auto steps = tss.solve(target);
            Assert::IsTrue(steps == 18);
        }
    };
}
