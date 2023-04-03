#pragma once
#include <array>
#include <deque>
#include <unordered_set>
#include <vector>
#include <map>
#include <functional>
#include <string>

#include "dijkstra/dijkstra.hpp"

namespace TimeSaver
{
	using Id = unsigned char;

	
	class Connection
	{
	public:
		enum class Direction : unsigned char
		{
			Forward = 0b0,
			Backward = 0b1
		};

		enum class TurnoutState : signed char
		{
			None = -1,
			A_B = 0,
			A_C = 1
		};

		inline Connection(const Id target, const Direction direction, const TurnoutState turnoutState = TurnoutState::None)
			: target(target), direction(direction), turnoutState(turnoutState)
		{

		}
		const Direction direction;
		const TurnoutState turnoutState;
		const Id target;

		inline const bool isTurnoutConnection() const
		{
			return this->turnoutState != TurnoutState::None;
		}

		inline const bool isTurnoutConnectionTo(const Id target) const
		{
			return this->target == target && this->isTurnoutConnection();
		}
	};
	using Connections = std::deque<Connection>;

	inline const Connection::TurnoutState other(const Connection::TurnoutState& state)
	{
		return state == Connection::TurnoutState::A_B ? Connection::TurnoutState::A_C : Connection::TurnoutState::A_B;
	}

	class Node
	{
	public:

		enum class Options : unsigned char
		{
			CanParkCars = 0b00100000
		};

		static const unsigned char optionMask = 0b11100000;
		static const unsigned char idMask = ~optionMask;

		Node() = delete;
		Node(const Id id, const Connections connections);
		const Id id;
		const Connections connections;
		const unsigned char options;

		const bool hasOption(const Options option) const
		{
			return options & (unsigned char)option;
		}

		inline const Connections otherDirectionConnections(const Connection::Direction than) const
		{
			Connections others;
			for (auto& connection : connections)
				if(connection.direction != than)
					others.push_back(connection);
			return others;
		}
		inline const Connections sameDirectionConnections(const Connection::Direction like) const
		{
			Connections others;
			for (auto& connection : connections)
				if (connection.direction == like)
					others.push_back(connection);
			return others;
		}

		const bool isTurnout() const
		{
			for (const auto& connection : connections)
				if (connection.turnoutState != Connection::TurnoutState::None)
					return true;
			return false;
		}

		const bool hasTurnoutConnectionTo(const Id that, Connection::TurnoutState &direction) const
		{
			for (const auto& connection : connections)
				if (connection.target == that && connection.turnoutState != Connection::TurnoutState::None)
				{
					direction = connection.turnoutState;
					return true;
				}
			return false;
		}

		const Connection& to(const Id other) const
		{
			for (auto& c : connections)
				if (c.target == other)
					return c;
		}

		unsigned int exits(Id& a, Id& b, const Connection::TurnoutState& direction) const
		{
			if (this->isTurnout())
			{
				for (auto& connection : connections)
				{
					if (connection.turnoutState == Connection::TurnoutState::None)
						a = connection.target;
					else if (connection.turnoutState == Connection::TurnoutState::A_B && direction == Connection::TurnoutState::A_B)
						b = connection.target;
					else if (connection.turnoutState == Connection::TurnoutState::A_C && direction == Connection::TurnoutState::A_C)
						b = connection.target;
				}
				return 2;
			}
			// bumper
			else if (this->connections.size() == 1)
			{
				a = this->connections[0].target;
				return 1;
			}
			// straight
			else
			{
				a = this->connections[0].target;
				b = this->connections[1].target;
				return 2;
			}

			return 0;
		}

		const bool next(const Id from, Id &leavingTo, const Connection::TurnoutState& direction) const
		{
			// turnout
			if (this->isTurnout())
			{
				/*
					a == b
				      \\ c
				*/
				Id a, b, c;
				for (auto& connection : connections)
				{
					if (connection.turnoutState == Connection::TurnoutState::None)
						a = connection.target;
					else if (connection.turnoutState == Connection::TurnoutState::A_B)
						b = connection.target;
					else
						c = connection.target;
				}
				for (auto& connection : connections)
				{
					if (connection.turnoutState == Connection::TurnoutState::None)
					{
						// from a
						if (connection.target == from)
						{
							leavingTo = (direction == Connection::TurnoutState::A_B ? b : c);
							return true;
						}
					}
					else // b or c
					{
						if (connection.target == from && connection.turnoutState == direction)
						{
							leavingTo = a;
							return true;
						}
					}
				}
			}
			// bumper
			else if(this->connections.size() == 1)
			{
				if(this->connections[0].target == from)
					return false;
				else
				{
					leavingTo = this->connections[0].target;
					return true;
				}
			}
			// straight
			else
			{
				leavingTo = this->connections[0].target == from ? this->connections[1].target : this->connections[0].target;
				return true;
			}

			return false;
		}
	};

#ifdef TSS_FLEXIBLE
	using Nodes = std::vector<Node>;
#else
	template<unsigned int _Count> using Nodes = std::array<Node, _Count>;
#endif

	inline Node::Node(const Id id, const Connections connections)
		: id(id & Node::idMask), connections(connections), options(id & Node::optionMask)
	{

	}

#ifdef TSS_FLEXIBLE
#else
	template<unsigned int _Nodes, unsigned int _Cars>
#endif
	class Solver
	{
	public:
		struct SimpleState
		{
#ifdef TSS_FLEXIBLE
			std::vector<unsigned int> slots;
			SimpleState(const std::vector<unsigned int>& slots) :
				slots(slots)
			{

			}
#else
			std::array<unsigned int, _Nodes> slots;
			SimpleState(const std::array<unsigned int, _Nodes>& slots) :
				slots(slots)
			{

			}
#endif
			SimpleState() : slots() { };
			SimpleState(const SimpleState& s) :
				slots(s.slots)
			{

			}
		};

		struct State : SimpleState
		{
			State() : SimpleState(), turnouts() { };
#ifdef TSS_FLEXIBLE
			std::vector<Connection::TurnoutState> turnouts;
#else
			std::array<Connection::TurnoutState, _Nodes> turnouts;
#endif
			State(const State& s) :
				SimpleState(s), turnouts(s.turnouts)
			{

			}

			State(std::vector<unsigned int> slots, std::vector<Connection::TurnoutState> turnouts) :
				SimpleState(slots), turnouts(turnouts)
			{

			}
		};

		struct Step
		{
			struct TurnoutConfiguration
			{
				TurnoutConfiguration() : configuration(0) { };
				void set(unsigned char i, Connection::TurnoutState state)
				{
#define SET_nth_BIT(data, bit, value) (data & ~(1 << bit)) | ((value) << bit)
					if (i < 7)
						this->configuration = (unsigned char)SET_nth_BIT(this->configuration, i, (unsigned char)state);
#undef SET_nth_BIT
				}
				Connection::TurnoutState get(unsigned char i) const
				{
#define GET_nth_BIT(data, bit)	((data >> bit) & 1)
					if (i < 7)
						return GET_nth_BIT(this->configuration, i) == 0 ? Connection::TurnoutState::A_B : Connection::TurnoutState::A_C;
					return Connection::TurnoutState::None;
#undef GET_nth_BIT
				}
				const bool equals(const TurnoutConfiguration& tc) const
				{
					return this->configuration == tc.configuration;
				}
			private:
				unsigned char configuration;
			};

			struct Action
			{
				const size_t target;
				const Connection::Direction locoDirection;
				const TurnoutConfiguration turnoutConfiguration;
				Action(const size_t target, const Connection::Direction locoDirection, const TurnoutConfiguration turnoutConfiguration)
					: target(target), locoDirection(locoDirection), turnoutConfiguration(turnoutConfiguration) { };
			
				Action operator =(const Action& other)
				{
					Action a(other.target, other.locoDirection, other.turnoutConfiguration);
					return a;
				}
			};
			bool endState;
			std::vector<Action> actions;

			Step() : actions(), endState{ false } { };
			Step(const bool endState) : actions(), endState{ endState } { };
			Step(std::vector<Action> actions)
				: actions(actions), endState{ false }
			{ };
			Step(std::vector<Action> actions, const bool endState)
				: actions(actions), endState{ endState }
			{ };

			Step operator=(Step other)
			{
				Step s(other);
				return s;
			}
			
			Step(const Step& s) :
				actions(s.actions), endState{ s.endState }
			{

			}

			const size_t addAction(Action action)
			{
				const size_t id = this->actions.size();
				this->actions.push_back(action);
				return id;
			}

			bool hasActionToOther(const Action& otherAction)
			{
				for (auto& action : this->actions)
					if (action.target == otherAction.target)
						return true;

				return false;
			}

			const Action& actionToTarget(const size_t target)
			{
				for (const auto& action : this->actions)
					if (action.target == target)
						return action;
			}

			const bool hasAction(const Action a)
			{
				for (const auto& action : this->actions)
					if (action.target == a.target && action.locoDirection == a.locoDirection && action.turnoutConfiguration.equals(a.turnoutConfiguration))
						return true;

				return false;
			}
		};

		template<typename _State>
		struct StepInternals : public Step
		{
			using Action = Step::Action;
			const _State state;
			
			StepInternals() : Step(), state() { };
			StepInternals(const _State state)
				: Step(), state(state)
			{ };
			StepInternals(const _State state, std::vector<Action> actions)
				: Step(actions), state(state)
			{ };
			StepInternals(const _State state, const bool endState)
				: Step(endState), state(state)
			{ };

			StepInternals operator=(StepInternals other)
			{
				StepInternals s(other);
				return s;
			}
			
			StepInternals(const StepInternals& s) :
				Step(s.actions, s.endState), state(s.state)
			{

			}
		};

		struct StepWithId : StepInternals<State>
		{
			const size_t id;
			StepWithId(const size_t id, const State state)
				: StepInternals<State>(state), id(id)
			{

			}
			StepWithId(const StepWithId& other)
				: StepInternals<State>(other), id(other.id)
			{

			}
			StepWithId operator =(const StepWithId& other)
			{
				StepWithId s(other);
				return s;
			}
		};

		using Dijk = Dijkstra</*Step, Steps,*/ unsigned long>;
		using FullStep = StepWithId;
		using SimpleStep = StepInternals<SimpleState>;
		using Steps = std::vector<FullStep>;
		using SimpleSteps = std::vector<SimpleStep>;

	public:
		enum class Result : unsigned int
		{
			OK,
			NoLoco,
			Deadlock,
			IncompatibleTypes
		};

#ifdef TSS_FLEXIBLE
		using CarPlacement = std::vector<unsigned int>;
#else
		using CarPlacement = std::array<unsigned int, _Cars>;
#endif
		using PrintCallback = std::function<void(const std::string, const SimpleState&, const typename Step::Action&)>;
		using GraphCreationCallback = std::function<void(const unsigned int step, const unsigned int steps, const unsigned int solutions)>;
		using StatisticsCallback = std::function<void(const unsigned int steps, const unsigned int solutions)>;

		using DistanceStorage = Dijk::DistanceStorage;
		using PrecStorage = Dijk::PrecStorage;

		struct ActionIndices
		{
			size_t step;
			size_t action;
		};

		Solver(
#ifdef TSS_FLEXIBLE
			const Nodes nodes,
#else
			const Nodes<_Nodes> nodes, 
#endif
			PrintCallback print, GraphCreationCallback creation, StatisticsCallback statistics, Dijk::DistanceStorage& dist, Dijk::PrecStorage& prec)
			: nodes(nodes), print(print), creation(creation), statistics(statistics), dijkstra(0xFFFFFFFF, dist, prec)
		{

		}

		Result init(Steps steps)
		{
			/*this->nextStepIndex = std::rand() % this->simpleSteps.size();
			for(unsigned int i = 0; i < this->steps[this->nextStepIndex].state.slots.size(); ++i)
				if (this->steps[this->nextStepIndex].state.slots[i] == 1)
				{
					loco = i;
					break;
				}*/

			return Result::OK;
		}

		Result init(CarPlacement cars)
		{
			State state;
#ifdef TSS_FLEXIBLE
			state.slots.resize(this->nodes.size());
			state.turnouts.resize(this->nodes.size());
#endif
			for (auto& t : state.turnouts)
				t = Connection::TurnoutState::A_B;

			for (auto& s : state.slots)
				s = 0;

			unsigned int carIndex = 0;
			for (auto& c : cars)
			{
				if (c >= state.slots.size())
					return Result::IncompatibleTypes;
				state.slots[c] = ++carIndex;
			}

			for (size_t n = 0; n < nodes.size(); ++n)
				if (nodes[n].isTurnout())
					this->turnoutIndices.push_back(n);

			loco = cars[0];
			this->simpleSteps.clear();
			this->nextActionIndex = this->actions.size();

			int nextIndex = this->hasStepWithState(state);
			if (nextIndex == -1)
				nextIndex = (int)this->addStep(state);

			return Result::OK;
		}

#ifdef TSS_FLEXIBLE
		const CarPlacement random(size_t _Cars) 
		{
			CarPlacement result(_Cars);
#else
		const CarPlacement random()
		{
			CarPlacement result;
#endif
			for (unsigned int i = 0; i < result.size(); ++i)
			{
				bool used = true;
				unsigned int p = std::rand() % nodes.size();
				while (used)
				{
					p = std::rand() % nodes.size();

					if (this->nodes[p].isTurnout())
						continue;

					used = false;
					for (unsigned int j = 0; j < result.size(); ++j)
						if (result[j] == p)
						{
							used = true;
							break;
						}
				}
				result[i] = p;
			}

			return result;
		}

		const CarPlacement randomFromStepsGraph()
		{
			CarPlacement result;
			auto i = std::rand() % this->steps.size();
			const size_t maxCars = this->steps[i].state.slots.size();
			for (unsigned int j = 0; j < maxCars; ++j)
				for (unsigned int k = 1; k < maxCars; ++k)
					if (this->steps[i].state.slots[j] == k)
						result[k - 1] = j;

			return result;
;		}

		void solve_init(CarPlacement cars)
		{
#ifdef TSS_FLEXIBLE
			targetState.slots.clear();
			targetState.turnouts.clear();
			targetState.slots.resize(this->nodes.size());
			targetState.turnouts.resize(this->nodes.size());
#endif
			for (auto& s : targetState.slots)
				s = 0;

			unsigned int carIndex = 0;
			for (auto& c : cars)
				targetState.slots[c] = ++carIndex;


			State initial;
			initial.slots = this->simpleSteps[0].state.slots;
			typename Step::TurnoutConfiguration tc;
			initial.turnouts = this->turnoutsStateFromTurnoutConfiguration(tc);
			this->addStep(initial);
			analyze(initial, 0);
		}

		bool analyze(const State& state, const size_t index)
		{
			const auto i = index;
			auto loco = this->findLoco(state);
			if (loco == -1)
				return false;

			auto node = nodes[loco];

			for (auto& connection : node.connections)
			{
				// loco can only follow set turnouts
				if (this->nodes[loco].isTurnout() && connection.isTurnoutConnection())
				{
					if (connection.turnoutState != state.turnouts[loco])
						continue;
				}
				// check if loco can move
				this->move(state, i, loco, connection);
			}
		}

		State stateFromAction(const typename Step::Action& action)
		{
			State state;

			state.slots = this->simpleSteps[action.target].state.slots;
			state.turnouts = turnoutsStateFromTurnoutConfiguration(action.turnoutConfiguration);

			return state;
		}

		bool solve_step(bool mayChooseSolutionIfImpossible = true)
		{
			/*const size_t i = this->nextStepIndex;
			auto& step = this->steps[i];
			auto loco = this->findLoco(step.state);
			if (loco == -1)
				return false;

			auto node = nodes[loco];

			for (auto& connection : node.connections)
			{
				// loco can only follow set turnouts
				if (this->nodes[loco].isTurnout() && connection.isTurnoutConnection())
				{
					if (connection.turnoutState != this->steps[i].state.turnouts[loco])
						continue;
				}
				// check if loco can move
				this->move(i, loco, connection);
			}

			if (this->steps[i].state.slots == targetState.slots)
			{
				++solutions;
				this->steps[i].endState = true;
			}

			creation(i, (unsigned int)this->steps.size(), 0);
			this->nextStepIndex += 1;

			if (this->nextStepIndex == this->steps.size() && solutions == 0 && mayChooseSolutionIfImpossible)
			{
				// get the cars position of this
				unsigned int i = std::rand() % this->steps.size();

				// mark all as valid with the same cars position
				for(auto &step : this->steps)
					if (step.state.slots == this->steps[i].state.slots)
					{
						step.endState = true;
						++solutions;
					}
			}

			return this->nextStepIndex < this->steps.size();*/
			const size_t i = this->nextActionIndex;
			auto& action = this->actions[i];

			auto state = stateFromAction(this->simpleSteps[action.step].actions[action.action]);
			analyze(state, this->simpleSteps[action.step].actions[action.action].target);
			/*if (this->steps[i].state.slots == targetState.slots)
			{
				++solutions;
				this->steps[i].endState = true;
			}*/

			creation(i, (unsigned int)this->actions.size(), 0);
			this->nextActionIndex += 1;

			if (this->nextActionIndex == this->actions.size() && solutions == 0 && mayChooseSolutionIfImpossible)
			{
				// get the cars position of this
				unsigned int i = std::rand() % this->simpleSteps.size();

				// mark all as valid with the same cars position
				for (auto& step : this->simpleSteps)
					if (step.state.slots == this->simpleSteps[i].state.slots)
					{
						step.endState = true;
						++solutions;
					}
			}
			

			return this->nextActionIndex < this->actions.size();
		}

		void solve_dijkstra_init()
		{
			//simplify();
			dijkstra.dijkstra_init(this->simpleSteps.size(), 0);
		}

		bool solve_dijkstra_step()
		{
			return dijkstra.dijkstra_step([&](const size_t i) -> const std::vector<size_t> {
				std::vector<size_t> neighbors;
				for (auto neighbor : simpleSteps[i].actions)
					neighbors.push_back(neighbor.target);
				return neighbors;
				},
				[&](const size_t a, const size_t b, Dijk::PrecStorage::GetCallback prec) -> const unsigned int {
					for (auto neighbor : simpleSteps[a].actions)
						if (neighbor.target == b)
							return (unsigned int)1;

					return 0;
				});
		}

		unsigned int solve_dijkstra_shortestPath()
		{
			typename Dijk::Path shortestPath;
			unsigned int currentSolutionCheck = 0;
			size_t shortestPathSteps = dijkstra.infinity;
			for (size_t i = 0; i < this->simpleSteps.size(); ++i)
			{
				const auto& step = this->simpleSteps[i];
				if (step.endState)
				{
					++currentSolutionCheck;
					//std::cout << "\r" << "Checking solution " << currentSolutionCheck << " / " << solutions;
					auto path = dijkstra.shortestPath(i);
					if (path.size() < shortestPathSteps)
					{
						shortestPath = path;
						shortestPathSteps = path.size();
					}
				}
			}
			//std::cout << "\nSolutions has: " << shortestPath.size() << " steps\n";
			statistics((unsigned int)this->simpleSteps.size(), solutions);
			if (solutions == 0)
				return 0;

			typename Step::TurnoutConfiguration tc;
			typename Step::Action initial(0, Connection::Direction::Forward, tc);
			print("Start\n", this->simpleSteps[0].state, initial);
			print("End\n", targetState, simpleSteps[shortestPath[1]].actionToTarget(0));
#ifdef _DEBUG
			for (auto i = (int)shortestPath.size() - 1; i >= 0; --i)
			{
				print(std::string("Step").append(std::to_string((unsigned int)(i))), simpleSteps[shortestPath[i]].state,
					i == 0 ? initial : simpleSteps[shortestPath[i-1]].actionToTarget(shortestPath[i]));
			}
#endif
			return (unsigned int)shortestPathSteps;
		}

		unsigned int solve(CarPlacement cars, bool mayChooseSolutionIfImpossible = true)
		{
			solve_init(cars);

			// build graph
			while (solve_step(mayChooseSolutionIfImpossible));

			solve_dijkstra_init();

			while (solve_dijkstra_step());

			return solve_dijkstra_shortestPath();
		}

		void importSteps(Steps steps)
		{
			this->steps = steps;
		}

#ifdef TSS_WITH_EXPORT
		void exportSteps(std::ostream& out, std::string name)
		{
			/*
				TimeSaver::Solver::Steps steps = {{
					{id, State{slots, turnouts}, Actions{Action{target, direction}}}
				}};

			*/

			out << "TimeSaver::Solver::Steps " << name << " {{\n";

			for (const auto step : steps)
			{
				// id
				out << "{" << step.id << ",";

				// State
				out << "{{";
					// slots
					for (const auto slot : step.state.slots)
						out << slot << ",";
				out << "},{";
					// turnouts
					for (const auto turnout : step.state.turnouts)
						out << (turnout == Connection::TurnoutState::None ? "TimeSaver::Connection::TurnoutState::None" : (turnout == Connection::TurnoutState::A_B ? "TimeSaver::Connection::TurnoutState::A_B" : "TimeSaver::Connection::TurnoutState::A_C")) << ",";
				out << "}},";
			
				// Actions
				out << "{";
				for (const auto action : step.actions)
					out << "{" << action.target << ", " << (action.locoDirection == Connection::Direction::Forward ? "TimeSaver::Connection::Direction::Forward" : "TimeSaver::Connection::Direction::Backward") << "},";
				out << "}";
				out << "},\n";
			}

			out << "}};\n";
		}
#endif

		const size_t steps_count()
		{
			return this->steps.size();
		}

		const size_t simpleStepWithState(const State& state)
		{
			for (size_t i = 0; i < this->simpleSteps.size(); ++i)
				if (this->simpleSteps[i].state.slots == state.slots)
					return i;

			return -1;
		}

		const size_t addSimpleStep(const FullStep& step)
		{
			SimpleState simpleState(step.state.slots);
			this->simpleSteps.emplace_back(simpleState, step.endState);
			return this->simpleSteps.size() - 1;
		}

		/*void simplify()
		{
			for (const auto step : this->steps)
			{
				auto i = simpleStepWithState(step.state);
				if (i == -1)
					i = addSimpleStep(step);
				/ rewrite action targets to match new ids, not the old ones /
				simpleSteps[i].actions.insert(simpleSteps[i].actions.end(), step.actions.begin(), step.actions.end());
			}
		}*/


#ifdef TSS_FLEXIBLE
#else
		using Nodes = Nodes<_Nodes>;
#endif

	private:

		const Connection outwardConnection(const State& state, const Id id, const Connection::Direction direction, bool& success) const
		{
			if (nodes[id].isTurnout())
			{
				for (const auto& connection : nodes[id].connections)
				{
					if (connection.direction == direction && 
						(connection.turnoutState == Connection::TurnoutState::None || connection.turnoutState == state.turnouts[id]))
					{
						success = true;
						return connection;
					}
				}
			}
			else
			{
				if (nodes[id].sameDirectionConnections(direction).size() > 0)
				{
					success = true;
					return nodes[id].sameDirectionConnections(direction)[0];
				}
			}

			success = false;
			return Connection(0, Connection::Direction::Forward);
		}

		void move(const State& stateIn, const size_t i, const Id from, const Connection connectionTo)
		{
			auto to = connectionTo.target;
			const Connection::Direction locoDirection = connectionTo.direction;
			//auto conditions = connectionTo.conditions;
			int lastNodedPushedOn = from;

			State state = /*this->steps[i].*/stateIn;
			bool pushedSomethingFromTo = state.slots[to] != Empty;

			if (state.slots[to] == Loco)
			{
				// cannot push the loco away
				return;
			}
			else if (state.slots[to] != Empty)
			{
				// push cars if possible and then this one
				bool success = false;
				auto connection = outwardConnection(state, to, connectionTo.direction, success);
				if (success)
					lastNodedPushedOn = push(state, to, connection);
			}

			pushedSomethingFromTo = pushedSomethingFromTo && state.slots[to] == Empty;

			if (state.slots[to] == Empty)
			{
				if (nodes[to].isTurnout())
				{
					Connection::TurnoutState turnoutDirection;
					// from == to == a
					//  other //
					if (nodes[to].hasTurnoutConnectionTo(from, turnoutDirection))
					{
						State nextState = state;
						if (!pushedSomethingFromTo || nextState.turnouts[to] == turnoutDirection)
						{
							nextState.turnouts[to] = turnoutDirection;
							auto updatedStepState = nextStep(nextState, from, to, i, locoDirection);
							auto connections = nodes[from].otherDirectionConnections(connectionTo.direction);
							pull(updatedStepState, from, connections, i, locoDirection);

							// we pushed the last car onto a turnout, so add the toggled pushedOnTurnout as well
							if (lastNodedPushedOn != -1 && lastNodedPushedOn != from && nodes[lastNodedPushedOn].isTurnout())
							{
								State nextState = state;
								nextState.turnouts[to] = turnoutDirection;
								nextState.turnouts[lastNodedPushedOn] = other(nextState.turnouts[lastNodedPushedOn]);
								auto updatedStepState = nextStep(nextState, from, to, i, locoDirection);
								auto connections = nodes[from].otherDirectionConnections(connectionTo.direction);
								pull(updatedStepState, from, connections, i, locoDirection);
							}
						}
					}
					else
					// otherA == to == from
					//   otherB //
					{
						for (const auto& connection : nodes[to].connections)
						{
							if (connection.target != from)
							{
								turnoutDirection = connection.turnoutState;
								State nextState = state;
								if (!pushedSomethingFromTo || nextState.turnouts[to] == turnoutDirection)
								{
									nextState.turnouts[to] = turnoutDirection;
									auto updatedStepState = nextStep(nextState, from, to, i, locoDirection);
									auto connections = nodes[from].otherDirectionConnections(connection.direction);
									pull(updatedStepState, from, connections, i, locoDirection);

									// we pushed the last car onto a turnout, so add the toggled pushedOnTurnout as well
									if (lastNodedPushedOn != -1 && lastNodedPushedOn != from && nodes[lastNodedPushedOn].isTurnout())
									{
										State nextState = state;
										nextState.turnouts[to] = turnoutDirection;
										nextState.turnouts[lastNodedPushedOn] = other(nextState.turnouts[lastNodedPushedOn]);
										auto updatedStepState = nextStep(nextState, from, to, i, locoDirection);
										auto connections = nodes[from].otherDirectionConnections(connection.direction);
										pull(updatedStepState, from, connections, i, locoDirection);
									}
								}
							}
						}
					}
				}
				else
				{
					auto updatedStepState = nextStep(state, from, to, i, locoDirection);
					auto connections = nodes[from].otherDirectionConnections(connectionTo.direction);
					pull(updatedStepState, from, connections, i, locoDirection);

					// we pushed the last car onto a turnout, so add the toggled pushedOnTurnout as well
					if (lastNodedPushedOn != -1 && lastNodedPushedOn != from && nodes[lastNodedPushedOn].isTurnout())
					{
						State nextState = state;
						nextState.turnouts[lastNodedPushedOn] = other(nextState.turnouts[lastNodedPushedOn]);
						auto updatedStepState = nextStep(nextState, from, to, i, locoDirection);
						auto connections = nodes[from].otherDirectionConnections(connectionTo.direction);
						pull(updatedStepState, from, connections, i, locoDirection);
					}
				}
			}
		}

		// pull cars onto from
		void pull(State fromState, const Id from, const Connections connections, const size_t attach, const Connection::Direction locoDirection)
		{
			for (const auto& connection : connections)
			{
				Connection targetConnection = nodes[connection.target].to(from);
				// targetConnection.target = from
				// connection.target = other direction of to

				State state = fromState;
				if (state.slots[connection.target] == Empty)
					continue;

				if (nodes[from].isTurnout())
				{
					if(connection.turnoutState == Connection::TurnoutState::None || connection.turnoutState == state.turnouts[from])
					{
						auto updatedStepState = nextStep(state, connection.target, targetConnection.target, attach, locoDirection);
						auto connections = nodes[connection.target].sameDirectionConnections(connection.direction);
						pull(updatedStepState, connection.target, connections, attach, locoDirection);
					}
				}
				else
				{
					auto updatedStepState = nextStep(state, connection.target, targetConnection.target, attach, locoDirection);
					auto connections = nodes[connection.target].sameDirectionConnections(connection.direction);
					pull(updatedStepState, connection.target, connections, attach, locoDirection);
				}
			}
		}

		bool checkValidState(const State& state)
		{
			//return true;
			// you must not leave your cars unattended!
			for (unsigned int i = 0; i < state.slots.size(); ++i)
			{
				if (nodes[i].isTurnout() && !(state.slots[i] == Empty || state.slots[i] == Loco) && !nodes[i].hasOption(Node::Options::CanParkCars))
				{
					bool foundLoco = false;
					struct Edge
					{
						Id from;
						Id to;
						Edge() : from(0), to(0) { };
						Edge(unsigned int from, unsigned int to) : from(from), to(to) { };
					};
					std::vector<Edge> edges;
					Edge A, B;
					A.from = i;
					B.from = i;
		
					// turnouts return 2
					if (nodes[i].exits(A.to, B.to, state.turnouts[i]) == 2)
					{
						edges.push_back(A);
						edges.push_back(B);
						for (unsigned int j = 0; j < edges.size(); ++j)
						{
							const Edge& edge = edges[j];
							if (state.slots[edge.to] == Loco)
							{
								foundLoco = true;
								break;
							}
							else if (state.slots[edge.to] == Empty)
								continue;

							Id id;
							if (nodes[edge.to].next(edge.from, id, state.turnouts[edge.to]))
								edges.emplace_back(edge.to, id);
						}
					}
					if (!foundLoco)
						return false;
				}
			}
			return true;
		}

		bool hasActionIndices(size_t step, size_t action)
		{
			for (const auto& indices : this->actions)
				if (indices.action == action && indices.step == step)
					return true;
			return false;
		}

		State nextStep(State state, const Id from, const Id to, const size_t attach, const Connection::Direction locoDirection)
		{
			state.slots[to] = state.slots[from];
			state.slots[from] = Empty;

			if (checkValidState(state))
			{
				int nextIndex = this->hasStepWithState(state);
				if (nextIndex == -1)
					nextIndex = (int)this->addStep(state);
				const auto tc = turnoutConfiguration(state);
				const auto action = typename Step::Action(nextIndex, locoDirection, tc);
				if (!this->simpleSteps[attach].hasAction(action))
				{
					auto actionId = this->simpleSteps[attach].addAction(action);
					if (!hasActionIndices(attach, actionId))
						this->actions.emplace_back(attach, actionId);
				}

				if (this->simpleSteps[nextIndex].state.slots == targetState.slots)
				{
					++solutions;
					this->simpleSteps[nextIndex].endState = true;
				}
			}
			return state;
		}

		const int push(State& state, const Id from, const Connection connectionTo)
		{
			int lastNodedPushedOn = -1;
			auto to = connectionTo.target;

			if (state.slots[to] == Loco)
				return lastNodedPushedOn;

			bool doPush = false;
			if (nodes[to].isTurnout())
			{
				Connection::TurnoutState turnoutDirection;
				// from == to == a
				//  other //
				if (nodes[to].hasTurnoutConnectionTo(from, turnoutDirection))
				{
					if (state.slots[to] == Empty)
						state.turnouts[to] = turnoutDirection;

					if (state.turnouts[to] == turnoutDirection)
					{
						doPush = true;
					}
					//else
						lastNodedPushedOn = -1;// return lastNodedPushedOn;*/
				}
				else
					// otherA == to == from
					//   otherB //
				{
					doPush = true;
					lastNodedPushedOn = to;
				}
			}
			else
			{
				doPush = true;
			}

			if (state.slots[to] != Empty && doPush)
			{
				// push cars if possible and then this one
				bool success = false;
				auto connection = outwardConnection(state, to, connectionTo.direction, success);
				if (success)
					lastNodedPushedOn = push(state, to, connection);
			}
			
			if (state.slots[to] == Empty)
			{
			//	if (lastNodedPushedOn == -1)
				//	lastNodedPushedOn = to;
				state.slots[to] = state.slots[from];
				state.slots[from] = Empty;
				//return lastNodedPushedOn;
			}
			return lastNodedPushedOn;
		}

		bool checkForSolutionState(const CarPlacement &cars, const State &state) const
		{
			for (unsigned int s = 0; s < state.slots.size(); ++s)
			{
				if (state.slots[s] == Empty)
					continue;
				else if (cars[state.slots[s]] == s)
					continue;
				else
					return false;
			}
			return true;
		}

		template<typename... Args>
		const size_t addStep(Args&&... args) {
			size_t id = this->simpleSteps.size();
			this->simpleSteps.emplace_back(/*id,*/ std::forward<Args>(args)...);
			return id;
		}

		int hasStepWithState(const State& state) const
		{
			for (unsigned int i = 0; i < simpleSteps.size(); ++i)
				if (simpleSteps[i].state.slots == state.slots)//&& steps[i].state.turnouts == state.turnouts)
					return i;
			return -1;
		}

		int findLoco(const State& state) const
		{
			for (unsigned int i = 0; i < state.slots.size(); ++i)
				if (state.slots[i] == Loco)
					return i;

			return -1;
		}

		const typename Step::TurnoutConfiguration turnoutConfiguration(const State& state)
		{
			typename Step::TurnoutConfiguration result;

			for (size_t t = 0; t < this->turnoutIndices.size(); ++t)
				result.set(t, state.turnouts[this->turnoutIndices[t]]);

			return result;
		}
#ifdef TSS_FLEXIBLE
		const std::vector<Connection::TurnoutState> turnoutsStateFromTurnoutConfiguration(const typename Step::TurnoutConfiguration tc)
		{
			std::vector<Connection::TurnoutState> turnouts;
			turnouts.resize(this->nodes.size());
#else
		const std::array<Connection::TurnoutState, _Nodes> turnoutsStateFromTurnoutConfiguration(const typename Step::TurnoutConfiguration tc)
		{
			std::array<Connection::TurnoutState, _Nodes> turnouts;
#endif
			for (size_t i = 0; i < turnouts.size(); ++i)
				turnouts[i] = Connection::TurnoutState::None;
			
			for (size_t t = 0; t < this->turnoutIndices.size(); ++t)
				turnouts[this->turnoutIndices[t]] = tc.get(t);

			return turnouts;
		}

		const unsigned int Loco = 1;
		const unsigned int Empty = 0;
		unsigned int loco = 0;
		std::vector<unsigned int> turnoutIndices;

		unsigned int solutions = 0;
		size_t nextActionIndex = 0;
		State targetState;
		Dijk dijkstra;

		const Nodes nodes;
		PrintCallback print;
		GraphCreationCallback creation;
		StatisticsCallback statistics;

		Steps steps;
		SimpleSteps simpleSteps;

		std::vector<ActionIndices> actions;
	};
}