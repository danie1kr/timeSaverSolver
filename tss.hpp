
#include <array>
#include <deque>
#include <unordered_set>
#include <vector>
#include <map>
#include <functional>

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
	};
	template<unsigned int _Count> using Nodes = std::array<Node, _Count>;

	inline Node::Node(const Id id, const Connections connections)
		: id(id), connections(connections)
	{

	}

	template<unsigned int _Nodes, unsigned int _Cars>
	class Solver
	{
	public:
		struct State
		{
			std::array<unsigned int, _Nodes> slots;
			std::array<Connection::TurnoutState, _Nodes> turnouts;
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

	public:
		enum class Result : unsigned int
		{
			OK,
			NoLoco,
			Deadlock
		};

		using CarPlacement = std::array<unsigned int, _Cars>;
		using PrintCallback = std::function<void(const unsigned int id, const State&)>;

		Solver(const Nodes<_Nodes> nodes, PrintCallback print)
			: nodes(nodes), print(print)
		{

		}

		Result init(CarPlacement cars)
		{
			State state;
			for (auto& t : state.turnouts)
				t = Connection::TurnoutState::A_B;

			for (auto& s : state.slots)
				s = 0;

			unsigned int carIndex = 0;
			for (auto& c : cars)
				state.slots[c] = ++carIndex;

			loco = cars[0];
			this->steps.clear();
			this->addStep(state);

			return Result::OK;
		}

		const CarPlacement random()
		{
			CarPlacement result;
			for (unsigned int i = 0; i < _Cars; ++i)
			{
				bool used = true;
				unsigned int p = std::rand() % _Nodes;
				while (used)
				{
					p = std::rand() % _Nodes;

					if (this->nodes[p].isTurnout())
						continue;

					used = false;
					for (unsigned int j = 0; j < _Cars; ++j)
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

		unsigned int solve(CarPlacement cars)
		{
			State targetState;
			for (auto& s : targetState.slots)
				s = 0;

			unsigned int carIndex = 0;
			for (auto& c : cars)
				targetState.slots[c] = ++carIndex;

			std::cout << "\n";
			unsigned int solutions = 0;

			// build graph
			for(unsigned int i = 0; i < this->steps.size(); ++i)
			{
				auto &step = this->steps[i];
				auto loco = this->findLoco(step.state);
				if (loco == -1)
					return 0;

				auto node = nodes[loco];
				
				for (auto& connection : node.connections)
				{
					// loco can only follow set turnouts
					if (this->nodes[loco].isTurnout() && connection.isTurnoutConnection())
					{
						if(connection.turnoutState != this->steps[i].state.turnouts[loco])
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

#ifdef _DEBUG
				if (i % 20 == 0 || i == this->steps.size() - 1)
#else
				if (i % 10000 == 0 || i == this->steps.size() - 1)
#endif
				
				{
					std::cout << "\r" << "Step " << i + 1 << " / " << this->steps.size() << " possible solutions: " << solutions;
				}
			}
			// full graph size
			std::cout << "\ncar placement variants: " << this->steps.size() << "\n";

			if (solutions == 0)
			{
				unsigned int i = std::rand() % this->steps.size();
				std::cout << "\nchoosing: " << i << " as endstate\n";
				this->steps[i].endState = true;
				solutions = 1;
			}

			// get shortest path
			using Dijk = Dijkstra<Step, Steps, unsigned int, 0xFFFFFFFF>;
			Dijk dijkstra;
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
					std::cout << "\r" << "Checking solution " << currentSolutionCheck << " / " << solutions;
					auto path = dijkstra.shortestPath(&step);
					if (path.size() < shortestPathSteps)
					{
						shortestPath = path;
						shortestPathSteps = path.size();
					}
				}
			std::cout << "\nSolutions has: " << shortestPath.size() << " steps\n";

			std::cout << "\nmemory usage: ~" << (this->steps.size() * (sizeof(Step) + 4 * sizeof(Node*) + sizeof(unsigned int))) << " bytes \n";

			if (solutions == 0)
				return 0;

			print(0, this->steps[0].state);
			print(0, targetState);
#ifdef _DEBUG
			for(auto it = shortestPath.rbegin(); it != shortestPath.rend(); ++it)
				print((unsigned int)(*it)->id, (*it)->state);
#endif
			return (unsigned int)shortestPathSteps;
		}

		using Nodes = Nodes<_Nodes>;

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
							auto nextId = nextStep(nextState, from, to, i);
							auto connections = nodes[from].otherDirectionConnections(connectionTo.direction);
							pull(nextId, from, connections, i);

							if (lastNodedPushedOn != from && nodes[lastNodedPushedOn].isTurnout())
							{
								State nextState = state;
								nextState.turnouts[to] = turnoutDirection;
								nextState.turnouts[lastNodedPushedOn] = other(nextState.turnouts[lastNodedPushedOn]);
								auto nextId = nextStep(nextState, from, to, i);
								auto connections = nodes[from].otherDirectionConnections(connectionTo.direction);
								pull(nextId, from, connections, i);
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
									auto nextId = nextStep(nextState, from, to, i);
									auto connections = nodes[from].otherDirectionConnections(connection.direction);
									pull(nextId, from, connections, i);

									if (lastNodedPushedOn != from && nodes[lastNodedPushedOn].isTurnout())
									{
										State nextState = state;
										nextState.turnouts[to] = turnoutDirection;
										nextState.turnouts[lastNodedPushedOn] = other(nextState.turnouts[lastNodedPushedOn]);
										auto nextId = nextStep(nextState, from, to, i);
										auto connections = nodes[from].otherDirectionConnections(connection.direction);
										pull(nextId, from, connections, i);
									}
								}
							}
						}
					}
				}
				else
				{
					auto nextId = nextStep(state, from, to, i);
					auto connections = nodes[from].otherDirectionConnections(connectionTo.direction);
					pull(nextId, from, connections, i);

					if (lastNodedPushedOn != from && nodes[lastNodedPushedOn].isTurnout())
					{
						State nextState = state;
						nextState.turnouts[lastNodedPushedOn] = other(nextState.turnouts[lastNodedPushedOn]);
						auto nextId = nextStep(nextState, from, to, i);
						auto connections = nodes[from].otherDirectionConnections(connectionTo.direction);
						pull(nextId, from, connections, i);
					}
				}
			}
		}

		// pull cars onto from
		void pull(const size_t i, const Id from, const Connections connections, const size_t attach)
		{
			for (const auto& connection : connections)
			{
				Connection targetConnection = nodes[connection.target].to(from);
				// targetConnection.target = from
				// connection.target = other direction of to

				State state = this->steps[i].state;
				if (state.slots[connection.target] == Empty)
					continue;

				if (nodes[from].isTurnout())
				{
					if(connection.turnoutState == state.turnouts[from])
					{
						auto nextId = nextStep(state, connection.target, targetConnection.target, attach);
						auto connections = nodes[connection.target].sameDirectionConnections(connection.direction);
						pull(nextId, connection.target, connections, attach);
					}
				}
				else
				{
					auto nextId = nextStep(state, connection.target, targetConnection.target, attach);
					auto connections = nodes[connection.target].sameDirectionConnections(connection.direction);
					pull(nextId, connection.target, connections, attach);
				}
			}
		}

		size_t nextStep(State state, const Id from, const Id to, const size_t attach)
		{
			state.slots[to] = state.slots[from];
			state.slots[from] = Empty;

			int nextIndex = this->hasStepWithState(state);
			if (nextIndex == -1)
				nextIndex = (int)this->addStep(state);
			this->steps[attach].addAction(typename Step::Action(nextIndex));

			return nextIndex;			
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

		const Nodes nodes;
		PrintCallback print;

		Steps steps;
	};
}