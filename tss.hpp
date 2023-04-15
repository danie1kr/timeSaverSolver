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

		enum class TurnoutState : unsigned char
		{
			None,
			A_B,
			A_C,
			DontCare
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
		if (state == Connection::TurnoutState::None || state == Connection::TurnoutState::DontCare)
			return state;
		return state == Connection::TurnoutState::A_B ? Connection::TurnoutState::A_C : Connection::TurnoutState::A_B;
	}

	class Node
	{
	public:

		enum class Options : unsigned char
		{
			CanParkCars = 0b0100000
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
		struct State
		{
#ifdef TSS_FLEXIBLE
			std::vector<unsigned int> slots;
			std::vector<Connection::TurnoutState> turnouts;

			State() : slots(), turnouts() { };
			State(const State& s) :
				slots(s.slots), turnouts(s.turnouts)
			{

			}

			State(std::vector<unsigned int> slots, std::vector<Connection::TurnoutState> turnouts) :
				slots(slots), turnouts(turnouts)
			{

			}
#else
			std::array<unsigned int, _Nodes> slots;
			std::array<Connection::TurnoutState, _Nodes> turnouts;
#endif
		};

		struct PackedState
		{
			PackedState(const State state, const unsigned int turnouts, const unsigned int cars) : turnouts(turnouts), cars(cars), data(PackedState::fromState(state, turnouts, cars)) {};
			PackedState(const std::array<unsigned char, 6> data) : turnouts(extractTurnouts(data)), cars(extractCars(data)), data(data) {};

			const Connection::TurnoutState turnoutState(const unsigned int turnout) const
			{
				const int64_t I = toInt(this->data);
				unsigned char t = (I >> (6 + cars * 5 + turnout * 2)) & 0b11;
				if (t == 0b00)
					return Connection::TurnoutState::A_B;
				else if (t == 0b01)
					return Connection::TurnoutState::A_C;
				else if(t == 0b10)
					return Connection::TurnoutState::DontCare;
				else
					return Connection::TurnoutState::None;
			}

			const unsigned int node(const unsigned int node) const
			{
				const int64_t I = toInt(this->data);
				for (size_t c = 1; c <= cars; ++c)
				{
					if (((I >> ((6 + (c - 1) * 5))) & 0b11111) == (node & 0b11111))
						return c;
				}

				return 0;
			}

			const std::array<unsigned char, 6> data;

		private:
			static const unsigned int extractTurnouts(const std::array<unsigned char, 6> data)
			{
				return data[0] & 0b111;
			}

			static const unsigned int extractCars(const std::array<unsigned char, 6> data)
			{
				return (data[0] >> 3) & 0b111;
			}

			static const int64_t toInt(const std::array<unsigned char, 6>& data)
			{
				int64_t i;
				for (unsigned int a = 0; a < data.size(); ++a)
					i |= data[a] << (a * 8);

				return i;
			}

			static const std::array<unsigned char, 6> fromState(const State state, const unsigned int turnouts, const unsigned int cars)
			{
				int64_t data = 0;

#define REQ_BITS(n) ceil(log(n) / log(2.0));

				data |= (turnouts & 0b111) << 0;
				data |= (cars & 0b111) << 3;

				for (size_t c = 1; c <= cars; ++c)
					for (size_t i = 0; i < state.slots.size(); ++i)
						if (state.slots[i] == c)
							data |= (i & 0b11111) << (6 + (c - 1) * 5);

				unsigned int turnout = 0;
				for (size_t t = 0; t < state.turnouts.size(); ++t)
					if (state.turnouts[t] != Connection::TurnoutState::None)
					{
						if (state.turnouts[t] == Connection::TurnoutState::A_B)
							data |= (int64_t)0b00 << (int64_t)(6 + cars * 5 + turnout * 2);
						else if (state.turnouts[t] == Connection::TurnoutState::A_C)
							data |= (int64_t)0b01 << (int64_t)(6 + cars * 5 + turnout * 2);
						else if (state.turnouts[t] == Connection::TurnoutState::DontCare)
							data |= (int64_t)0b10 << (int64_t)(6 + cars * 5 + turnout * 2);

						++turnout;
					}

				std::array<unsigned char, 6> arr;
				for (unsigned int i = 0; i < arr.size(); ++i)
					arr[i] = (data >> (i * 8)) & 0xFF;

				return arr;
			}
			const unsigned int turnouts;
			const unsigned int cars;
		};

		struct PackedStep
		{
			struct PackedAction
			{
				const int32_t target() const { return data >> 1; }
				const Connection::Direction locoDirection() const { return (data & 0b1) == 0 ? Connection::Direction::Forward : Connection::Direction::Backward; }
				PackedAction(const int32_t data) : data(data) { };
				PackedAction(const int32_t target, const Connection::Direction locoDirection)
					: data(target << 1 | ((locoDirection == Connection::Direction::Forward) ? 0 : 1)) { };

				const int32_t data;
			};
			PackedStep(const PackedStep & step) : state{ step.state }, actions{ step.actions } {};

			PackedStep(const PackedState state, const std::vector<PackedAction> actions) : state{ state }, actions{ actions } {};

			PackedStep operator=(const PackedStep &other)
			{
				PackedStep s(other);
				return s;
			}

			const PackedState state;
			const std::vector<PackedAction> actions;
		};

		struct Step
		{
			struct Action
			{
				const size_t target;
				const Connection::Direction locoDirection;
				Action(const size_t target, const Connection::Direction locoDirection)
					: target(target), locoDirection(locoDirection) { };
			};

			const size_t id;
			const State state;
			std::vector<Action> actions;
			bool endState;
			
			Step(const size_t id) : id(id), state(), actions(), endState{ false } { };
			Step(const size_t id, const State state)
				: id(id), state(state), actions(), endState { false }
			{ };
			Step(const size_t id, const State state, std::vector<Action> actions)
				: id(id), state(state), actions(actions), endState{ false }
			{ };

			Step operator=(Step other)
			{
				Step s(other);
				return s;
			}

			Step(const Step& s) :
				id(s.id), state(s.state), actions(s.actions), endState{s.endState}
			{

			}

			bool hasActionToOther(const Action & otherAction)
			{
				for (auto& action : this->actions)
					if (action.target == otherAction.target)
						return true;

				return false;
			}

			void addAction(Action action)
			{
				this->actions.push_back(action);
			}
		};
		using Dijk = Dijkstra</*Step, Steps,*/ unsigned long>;
		using Steps = std::vector<Step>;
		using PackedSteps = std::vector<PackedStep>;

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
		using PrintCallback = std::function<void(const std::string, const PackedState&)>;
		using GraphCreationCallback = std::function<void(const unsigned int step, const unsigned int steps, const unsigned int solutions)>;
		using StatisticsCallback = std::function<void(const unsigned int steps, const unsigned int solutions)>;

		using DistanceStorage = Dijk::DistanceStorage;
		using PrecStorage = Dijk::PrecStorage;

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

		Result init(PackedSteps steps)
		{
			this->nextStepIndex = std::rand() % this->packedSteps.size();
			/*for (unsigned int i = 0; i < this->packedSteps[this->nextStepIndex].state.slots.size(); ++i)
				if (this->steps[this->nextStepIndex].state.slots[i] == 1)
				{
					loco = i;
					break;
				}*/

			return Result::OK;
		}

		Result init(CarPlacement cars, const bool keep = false)
		{
			State state;
#ifdef TSS_FLEXIBLE
			state.slots.resize(this->nodes.size());
			state.turnouts.resize(this->nodes.size());
#endif
			for (size_t i = 0; i < this->nodes.size(); ++i)
				state.turnouts[i] = this->nodes[i].isTurnout() ? Connection::TurnoutState::A_B : Connection::TurnoutState::None;

			for (auto& s : state.slots)
				s = 0;

			unsigned int carIndex = 0;
			for (auto& c : cars)
			{
				if (c >= state.slots.size())
					return Result::IncompatibleTypes;
				state.slots[c] = ++carIndex;
			}

			loco = cars[0];
			if(!keep)
				this->steps.clear();
			this->nextStepIndex = this->steps.size();

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
		}

		bool solve_step(bool mayChooseSolutionIfImpossible = true)
		{
			const size_t i = this->nextStepIndex;
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

			return this->nextStepIndex < this->steps.size();
		}

		void solve_dijkstra_init()
		{
			dijkstra.dijkstra_init(this->packedSteps.size(), 0);
		}

		bool solve_dijkstra_step()
		{
			return dijkstra.dijkstra_step([&](const size_t i) -> const std::vector<size_t> {
				std::vector<size_t> neighbors;
				for (auto neighbor : packedSteps[i].actions)
					neighbors.push_back(neighbor.target());
				return neighbors;
				},
				[&](const size_t a, const size_t b, Dijk::PrecStorage::GetCallback prec) -> const unsigned int {
					// look through prec of a to decide how man turnouts changed
					for (auto neighbor : packedSteps[a].actions)
						if (neighbor.target() == b)
							return (unsigned int)1;

					return 0;
				});
		}

		unsigned int solve_dijkstra_shortestPath()
		{
			typename Dijk::Path shortestPath;
			unsigned int currentSolutionCheck = 0;
			size_t shortestPathSteps = dijkstra.infinity;
			for (const size_t i : this->endStates)
			{
				auto& step = this->packedSteps[i];
				++currentSolutionCheck;
				//std::cout << "\r" << "Checking solution " << currentSolutionCheck << " / " << solutions;
				auto path = dijkstra.shortestPath(i);
				if (path.size() < shortestPathSteps)
				{
					shortestPath = path;
					shortestPathSteps = path.size();
				}
			}
			//std::cout << "\nSolutions has: " << shortestPath.size() << " steps\n";
			statistics((unsigned int)this->packedSteps.size(), solutions);
			if (solutions == 0)
				return 0;

			print("Start\n", this->packedSteps[0].state);

			PackedState packedTargetState(targetState, countTurnouts(), countCars());
			print("End\n", packedTargetState);
#ifdef _DEBUG
			for (auto it = shortestPath.rbegin(); it != shortestPath.rend(); ++it)
				print(std::string("Step").append(std::to_string((unsigned int)(*it))), packedSteps[(*it)].state);
#endif
			return (unsigned int)shortestPathSteps;
		}

		unsigned int solve(CarPlacement cars, bool mayChooseSolutionIfImpossible = true)
		{
			if (this->nextStepIndex == 0)
			{
				solve_init(cars);

				// build graph
				while (solve_step(mayChooseSolutionIfImpossible));

				pack();
			}
			solve_dijkstra_init();

			while (solve_dijkstra_step());

			return solve_dijkstra_shortestPath();
		}

		const unsigned int countTurnouts()
		{
			unsigned int turnouts = 0;
			for (const auto& node : this->nodes)
				if (node.isTurnout())
					++turnouts;

			return turnouts;
		}

		const unsigned int countCars()
		{
			unsigned int cars = 0;
			for (unsigned int i = 0; i < this->steps[0].state.slots.size(); ++i)
				if (this->steps[0].state.slots[i] != Empty)
					++cars;

			return cars;
		}

		void pack()
		{
			unsigned int turnouts = countTurnouts(), cars = countCars();

			for (const auto& step : this->steps)
			{
				const PackedState state(step.state, turnouts, cars);

				std::vector<PackedStep::PackedAction> actions;
				for (const auto& action : step.actions)
					actions.emplace_back(action.target, action.locoDirection);

				this->packedSteps.emplace_back(state, actions);

				if (step.endState)
					this->endStates.push_back(step.id);
			}
		}

		void importSteps(PackedSteps steps)
		{
			this->packedSteps = steps;
		}

#ifdef TSS_WITH_EXPORT
		void exportSteps(std::ostream& out, std::string name)
		{
			/*
				TimeSaver::Solver::Steps steps = {{
					{id, State{slots, turnouts}, Actions{Action{target, direction}}}
				}};
			*/			

			out << "TimeSaver::Solver::PackedSteps " << name << " {{\n";

			for (const auto step : packedSteps)
			{
				// State
				out << "{ {{";
					// slots
					for (const auto data : step.state.data)
						out << "0x" << std::hex << std::setfill('0') << std::setw(2) << (int)data << ",";
				out << "}},";
			
				// Actions
				out << "{{";
					for (const auto action : step.actions)
						out << "0x" << std::hex << std::setfill('0') << std::setw(2) << action.data << ",";
				out << "}} },\n";
			}

			out << "}};\n";
		}
#endif

		const size_t steps_count()
		{
			return this->packedSteps.size();
		}


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
						// potential problem with connection.turnoutState == Connection::TurnoutState::DontCare ?
						(connection.turnoutState == Connection::TurnoutState::None || state.turnouts[id] == Connection::TurnoutState::DontCare ||  connection.turnoutState == state.turnouts[id]))
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

		void move(const size_t i, const Id from, const Connection connectionTo)
		{
			auto to = connectionTo.target;
			const Connection::Direction locoDirection = connectionTo.direction;
			//auto conditions = connectionTo.conditions;
			int lastNodedPushedOn = from;

			State state = this->steps[i].state;
			bool pushedSomethingFromTo = state.slots[to] != Empty;

			PushedStates pushedStates;
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
					pushedStates = push(state, to, connection);
			}
			else
				pushedStates = {{ { lastNodedPushedOn, state } }};

			for (const auto& ps : pushedStates)
			{
				const auto lastNodedPushedOn = ps.lastNodePushedOn;
				const auto state = ps.state;
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
							if (!pushedSomethingFromTo || nextState.turnouts[to] == turnoutDirection || nextState.turnouts[to] == Connection::TurnoutState::DontCare)
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
									if (!pushedSomethingFromTo || nextState.turnouts[to] == turnoutDirection || nextState.turnouts[to] == Connection::TurnoutState::DontCare)
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
					if(connection.turnoutState == Connection::TurnoutState::None ||/* connection.turnoutState == Connection::TurnoutState::DontCare || state.turnouts[from] == Connection::TurnoutState::DontCare ||*/ connection.turnoutState == state.turnouts[from])
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

		State nextStep(State state, const Id from, const Id to, const size_t attach, const Connection::Direction locoDirection)
		{
			state.slots[to] = state.slots[from];
			state.slots[from] = Empty;

			//das hier ggf weg ? wenn man von einem turnout runter fährt, kann man keinen wagen mitnehmen-- >
			//	hier wird es auf don;t care gestzt, aber bei pull danach weiter benutzt. ggf. dort zurücksetzen?
			//if(!this->nodes[from].isTurnout())

			if (checkValidState(state))
			{
				auto putState = state;
				updateDontCare(putState);

				int nextIndex = this->hasStepWithState(putState);
				if (nextIndex == -1)
					nextIndex = (int)this->addStep(putState);
				this->steps[attach].addAction(typename Step::Action(nextIndex, locoDirection));
			}
			// futher pulls/pushes need the state without dontCare
			return state;
		}

		struct PushedState
		{
			const int lastNodePushedOn;
			const State state;
			PushedState(const PushedState& other)
				: lastNodePushedOn(other.lastNodePushedOn), state(other.state) {};
			PushedState(const int lastNodePushedOn, const State state)
				: lastNodePushedOn(lastNodePushedOn), state(state) { };
			PushedState operator=(const PushedState& other)
			{
				PushedState s(other);
				return s;
			}
		};
		using PushedStates = std::vector<PushedState>;

		const PushedStates push(State state, const Id from, const Connection connectionTo)
		{
			int lastNodedPushedOn = -1;
			auto to = connectionTo.target;

			if (state.slots[to] == Loco)
				return {{ {lastNodedPushedOn, state} }};

			bool doPush = false;
			PushedStates thisVals;
			if (nodes[to].isTurnout())
			{
				Connection::TurnoutState turnoutDirection;
				// from == to == a
				//  other //
				if (nodes[to].hasTurnoutConnectionTo(from, turnoutDirection))
				{
					if (state.slots[to] == Empty)
						state.turnouts[to] = turnoutDirection;

					if (state.turnouts[to] == turnoutDirection)// || state.turnouts[to] == Connection::TurnoutState::DontCare)
					{
						state.turnouts[to] = turnoutDirection;
						doPush = true;
					}
					//else
						lastNodedPushedOn = -1;// return lastNodedPushedOn;*/

					thisVals.emplace_back(lastNodedPushedOn, state);
				}
				else
					// otherA == to == from
					//   otherB //
				{
					doPush = true;
					lastNodedPushedOn = to;
					if (state.turnouts[to] == Connection::TurnoutState::DontCare)
					{
						{
							auto newState = state;
							newState.turnouts[to] = Connection::TurnoutState::A_B;
							thisVals.emplace_back(lastNodedPushedOn, newState);
						}
						{
							auto newState = state;
							newState.turnouts[to] = Connection::TurnoutState::A_C;
							thisVals.emplace_back(lastNodedPushedOn, newState);
						}
					}
					else
					{
						thisVals.emplace_back(lastNodedPushedOn, state);
					}
				}
			}
			else
			{
				doPush = true;
				thisVals.emplace_back(lastNodedPushedOn, state);
			}

			PushedStates retVals;
			for (const auto ps : thisVals)
			{
				auto state = ps.state;

				if (state.slots[to] != Empty && doPush)
				{
					// push cars if possible and then this one
					bool success = false;
					auto connection = outwardConnection(state, to, connectionTo.direction, success);
					if (success)
					{
						for (const auto& nps : push(state, to, connection))
						{
							auto state = nps.state;
							if (state.slots[to] == Empty)
							{
								//	if (lastNodedPushedOn == -1)
									//	lastNodedPushedOn = to;

								state.slots[to] = state.slots[from];
								state.slots[from] = Empty;
								//return lastNodedPushedOn;
							}
							retVals.emplace_back(nps.lastNodePushedOn, state);
						}
					}
				}
				else if (state.slots[to] == Empty)
				{
					//	if (lastNodedPushedOn == -1)
						//	lastNodedPushedOn = to;

					state.slots[to] = state.slots[from];
					state.slots[from] = Empty;
					retVals.emplace_back(ps.lastNodePushedOn, state);
					//return lastNodedPushedOn;
				}
				else
					retVals.emplace_back(ps.lastNodePushedOn, state);
			}
			return retVals;
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
			size_t id = this->steps.size();
			this->steps.emplace_back(id, std::forward<Args>(args)...);
			return id;
		}

		void updateDontCare(State& state) const
		{
			for (unsigned int i = 0; i < state.turnouts.size(); ++i)
				if (this->nodes[i].isTurnout() && state.slots[i] == Empty)
					state.turnouts[i] = Connection::TurnoutState::DontCare;
		}

		int hasStepWithState(const State& state) const
		{
			for (unsigned int i = 0; i < steps.size(); ++i)
				if (steps[i].state.slots == state.slots && steps[i].state.turnouts == state.turnouts)
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

		const unsigned int Loco = 1;
		const unsigned int Empty = 0;
		unsigned int loco = 0;

		unsigned int solutions = 0;
		size_t nextStepIndex = 0;
		State targetState;
		Dijk dijkstra;

		const Nodes nodes;
		PrintCallback print;
		GraphCreationCallback creation;
		StatisticsCallback statistics;

		Steps steps;
		PackedSteps packedSteps;
		std::vector<size_t> endStates;
	};
}
