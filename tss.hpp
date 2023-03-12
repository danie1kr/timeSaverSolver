
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
			A_C
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
		Node() = delete;
		Node(const Id id, const Connections connections);
		const Id id;
		const Connections connections;

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
		: id(id), connections(connections)
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
#else
			std::array<unsigned int, _Nodes> slots;
			std::array<Connection::TurnoutState, _Nodes> turnouts;
#endif
		};

	private:
		struct Step
		{
			struct Action
			{
				const size_t target;
				Action(const size_t target)
					: target(target) { };
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
		using Steps = std::vector<Step>;
		using Dijk = Dijkstra<Step, Steps, unsigned int>;

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
		using PrintCallback = std::function<void(const std::string, const State&)>;
		using GraphCreationCallback = std::function<void(const unsigned int step, const unsigned int steps, const unsigned int solutions)>;
		using StatisticsCallback = std::function<void(const unsigned int steps, const unsigned int solutions)>;

		Solver(
#ifdef TSS_FLEXIBLE
			const Nodes nodes,
#else
			const Nodes<_Nodes> nodes, 
#endif
			PrintCallback print, GraphCreationCallback creation, StatisticsCallback statistics)
			: nodes(nodes), print(print), creation(creation), statistics(statistics), dijkstra(0xFFFFFFFF)
		{

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

			loco = cars[0];
			this->steps.clear();
			this->addStep(state);

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

			this->nextStepIndex = 0;
		}

		bool solve_step(bool mayChooseSolutionIfImpossible = true)
		{
			const unsigned int i = this->nextStepIndex;
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
				unsigned int i = std::rand() % this->steps.size();
				this->steps[i].endState = true;
				solutions = 1;
			}

			return this->nextStepIndex < this->steps.size();
		}

		void solve_dijkstra_init()
		{
			dijkstra.dijkstra_init(this->steps, &this->steps[0]);
		}

		bool solve_dijkstra_step()
		{
			return dijkstra.dijkstra_step([&](const Step* node) -> const std::vector<const Step*> {
				std::vector<const Step*> neighbors;
				for (auto neighbor : node->actions)
					neighbors.push_back(&steps[neighbor.target]);
				return neighbors;
				},
				[](const Step* a, const Step* b) -> const unsigned int {
					for (auto neighbor : a->actions)
						if (neighbor.target == b->id)
							return (unsigned int)1;

					return 0;
				});
		}

		unsigned int solve_dijkstra_shortestPath()
		{
			typename Dijk::Path shortestPath;
			unsigned int currentSolutionCheck = 0;
			size_t shortestPathSteps = dijkstra.infinity;
			for (auto& step : this->steps)
				if (step.endState)
				{
					++currentSolutionCheck;
					//std::cout << "\r" << "Checking solution " << currentSolutionCheck << " / " << solutions;
					auto path = dijkstra.shortestPath(&step);
					if (path.size() < shortestPathSteps)
					{
						shortestPath = path;
						shortestPathSteps = path.size();
					}
				}
			//std::cout << "\nSolutions has: " << shortestPath.size() << " steps\n";
			statistics((unsigned int)this->steps.size(), solutions);
			if (solutions == 0)
				return 0;

			print("Start\n", this->steps[0].state);
			print("End\n", targetState);
#ifdef _DEBUG
			for (auto it = shortestPath.rbegin(); it != shortestPath.rend(); ++it)
				print(std::string("Step").append(std::to_string((unsigned int)(*it)->id)), (*it)->state);
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
			/*
			// get shortest path
			dijkstra.dijkstra(this->steps, &this->steps[0],
				[&](const Step* node) -> const std::vector<const Step*> {
					std::vector<const Step*> neighbors;
					for (auto neighbor : node->actions)
						neighbors.push_back(&steps[neighbor.target]);
					return neighbors;
				},
				[](const Step* a, const Step* b) -> const unsigned int {
					for (auto neighbor : a->actions)
						if (neighbor.target == b->id)
							return (unsigned int)1;

					return 0;
				});
			typename Dijk::Path shortestPath;
			unsigned int currentSolutionCheck = 0;
			size_t shortestPathSteps = 0xFFFFFFFF;
			for (auto& step : this->steps)
				if (step.endState)
				{
					++currentSolutionCheck;
					//std::cout << "\r" << "Checking solution " << currentSolutionCheck << " / " << solutions;
					auto path = dijkstra.shortestPath(&step);
					if (path.size() < shortestPathSteps)
					{
						shortestPath = path;
						shortestPathSteps = path.size();
					}
				}
			//std::cout << "\nSolutions has: " << shortestPath.size() << " steps\n";
			statistics((unsigned int)this->steps.size(), solutions);
			if (solutions == 0)
				return 0;

			print("Start\n", this->steps[0].state);
			print("End\n", targetState);
#ifdef _DEBUG
			for(auto it = shortestPath.rbegin(); it != shortestPath.rend(); ++it)
				print(std::string("Step").append(std::to_string((unsigned int)(*it)->id)), (*it)->state);
#endif
			return (unsigned int)shortestPathSteps;*/
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
					if (connection.direction == direction&& connection.turnoutState == state.turnouts[id])
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
			//auto conditions = connectionTo.conditions;
			auto lastNodedPushedOn = from;

			State state = this->steps[i].state;
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
							auto updatedStepState = nextStep(nextState, from, to, i);
							auto connections = nodes[from].otherDirectionConnections(connectionTo.direction);
							pull(updatedStepState, from, connections, i);

							if (lastNodedPushedOn != from && nodes[lastNodedPushedOn].isTurnout())
							{
								State nextState = state;
								nextState.turnouts[to] = turnoutDirection;
								nextState.turnouts[lastNodedPushedOn] = other(nextState.turnouts[lastNodedPushedOn]);
								auto updatedStepState = nextStep(nextState, from, to, i);
								auto connections = nodes[from].otherDirectionConnections(connectionTo.direction);
								pull(updatedStepState, from, connections, i);
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
									auto updatedStepState = nextStep(nextState, from, to, i);
									auto connections = nodes[from].otherDirectionConnections(connection.direction);
									pull(updatedStepState, from, connections, i);

									if (lastNodedPushedOn != from && nodes[lastNodedPushedOn].isTurnout())
									{
										State nextState = state;
										nextState.turnouts[to] = turnoutDirection;
										nextState.turnouts[lastNodedPushedOn] = other(nextState.turnouts[lastNodedPushedOn]);
										auto updatedStepState = nextStep(nextState, from, to, i);
										auto connections = nodes[from].otherDirectionConnections(connection.direction);
										pull(updatedStepState, from, connections, i);
									}
								}
							}
						}
					}
				}
				else
				{
					auto updatedStepState = nextStep(state, from, to, i);
					auto connections = nodes[from].otherDirectionConnections(connectionTo.direction);
					pull(updatedStepState, from, connections, i);

					if (lastNodedPushedOn != from && nodes[lastNodedPushedOn].isTurnout())
					{
						State nextState = state;
						nextState.turnouts[lastNodedPushedOn] = other(nextState.turnouts[lastNodedPushedOn]);
						auto updatedStepState = nextStep(nextState, from, to, i);
						auto connections = nodes[from].otherDirectionConnections(connectionTo.direction);
						pull(updatedStepState, from, connections, i);
					}
				}
			}
		}

		// pull cars onto from
		void pull(State fromState, const Id from, const Connections connections, const size_t attach)
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
					if(connection.turnoutState == state.turnouts[from])
					{
						auto updatedStepState = nextStep(state, connection.target, targetConnection.target, attach);
						auto connections = nodes[connection.target].sameDirectionConnections(connection.direction);
						pull(updatedStepState, connection.target, connections, attach);
					}
				}
				else
				{
					auto updatedStepState = nextStep(state, connection.target, targetConnection.target, attach);
					auto connections = nodes[connection.target].sameDirectionConnections(connection.direction);
					pull(updatedStepState, connection.target, connections, attach);
				}
			}
		}

		bool checkValidState(const State& state)
		{
			// you must not leave your cars unattended!
			for (unsigned int i = 0; i < state.slots.size(); ++i)
			{
				if (nodes[i].isTurnout() && !(state.slots[i] == Empty || state.slots[i] == Loco))
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

		State nextStep(State state, const Id from, const Id to, const size_t attach)
		{
			state.slots[to] = state.slots[from];
			state.slots[from] = Empty;

			if (checkValidState(state))
			{
				int nextIndex = this->hasStepWithState(state);
				if (nextIndex == -1)
					nextIndex = (int)this->addStep(state);
				this->steps[attach].addAction(typename Step::Action(nextIndex));
			}
			return state;
		}

		const Id push(State& state, const Id from, const Connection connectionTo)
		{
			auto lastNodedPushedOn = from;
			auto to = connectionTo.target;
			//auto conditions = connectionTo.conditions;
			if (state.slots[to] == Loco)
				return lastNodedPushedOn;
			else if (state.slots[to] != Empty)
			{
				bool doPush = false;
				if (nodes[to].isTurnout())
				{
					Connection::TurnoutState turnoutDirection;
					// from == to == a
					//  other //
					if (nodes[to].hasTurnoutConnectionTo(from, turnoutDirection))
					{
						if (state.turnouts[to] == turnoutDirection)
						{
							doPush = true;
						}
						else
							return lastNodedPushedOn;
					}
					else
					// otherA == to == from
					//   otherB //
					{
						doPush = true;
					}
				}
				else
				{
					doPush = true;
				}

				if(doPush)
				{
					// push cars if possible and then this one
					bool success = false;
					auto connection = outwardConnection(state, to, connectionTo.direction, success);
					if (success)
						lastNodedPushedOn = push(state, to, connection);
				}
			}
			
			if (state.slots[to] == Empty)
			{
				if (lastNodedPushedOn == from)
					lastNodedPushedOn = to;
				state.slots[to] = state.slots[from];
				state.slots[from] = Empty;
				return lastNodedPushedOn;
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
			size_t id = this->steps.size();
			this->steps.emplace_back(id, std::forward<Args>(args)...);
			return id;
		}

		size_t addStep(Step step)
		{
			this->steps.push_back(step);
			return this->steps.size() - 1;
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
		unsigned int nextStepIndex = 0;
		State targetState;
		Dijk dijkstra;

		const Nodes nodes;
		PrintCallback print;
		GraphCreationCallback creation;
		StatisticsCallback statistics;

		Steps steps;
	};
}