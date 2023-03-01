
#include <array>
#include <deque>
#include <unordered_set>
#include <vector>
#include <map>
#include <functional>

namespace TimeSaver
{
	using Id = unsigned char;

	template<class _Node, class _Nodes, typename _Distance, typename _Distance _Infinity>
	class Dijkstra
	{
		const _Distance infinity = _Infinity;
		
		std::vector<const _Node*> Q;
		std::map<const _Node*, _Distance> dist;
		std::map<const _Node*, const _Node*> prec;

	public:
		using Path = std::vector<const _Node*>;
		const Path shortestPath(const _Node* target)
		{
			Path path;
			path.push_back(target);
			const _Node* current = target;
			while (prec[current] != nullptr)
			{
				current = prec[current];
				path.push_back(current);
			}

			return path;
		}

		void dijkstra(const _Nodes &nodes, const _Node *start, std::function<const std::vector<const _Node*>(const _Node *)> neighbors, std::function<const _Distance(const _Node*, const _Node*)> distance)
		{
			dist.clear();
			prec.clear();
			Q.clear();

			// init
			for (auto it = nodes.begin(); it != nodes.end(); ++it)
			{
				dist.emplace(&*it, infinity);
				prec.emplace(&*it, nullptr);
				Q.push_back(&*it);
			}
			dist[start] = 0;

			// dijkstra
			while (Q.size() > 0)
			{
				const _Node* u;
				_Distance shortestDistInQ = _Infinity;
				for (auto it = Q.begin(); it != Q.end(); ++it)
				{
					if (dist[*it] < shortestDistInQ)
					{
						u = *it;
						shortestDistInQ = dist[*it];
					}
				}

				(void)std::remove(Q.begin(), Q.end(), u);
				Q.resize(Q.size() - 1);

				for (const auto v : neighbors(u))
				{
					if (std::find(Q.begin(), Q.end(), v) != Q.end())
					{
						_Distance dist_u_v = dist[u] + distance(u, v);
						if (dist[u] + dist_u_v < dist[v])
						{
							dist[v] = dist[u] + dist_u_v;
							prec[v] = u;
						}
					}
				}               
			}
		}
	};
	
	class Condition
	{
	public:
		enum class Type : unsigned char
		{
			Drive,
			Turnout
		};
		Condition() = delete;
		Condition(const Type type, const unsigned char data);
		virtual ~Condition() = default;
		const Type type;

	protected:
		const unsigned char data;
	};
	using Conditions = std::deque<Condition>;
	Condition::Condition(const Type type, const unsigned char data)
		: type(type), data(data)
	{

	};

	class TurnoutCondition : public Condition
	{
	public:
		enum class Direction : unsigned char
		{
			A_B = 0b0000000,
			A_C = 0b10000000
		};
		TurnoutCondition(const Id turnout, const Direction direction);
		const Id turnout() const;
		const Direction direction() const;
	};
	TurnoutCondition::TurnoutCondition(const Id turnout, const Direction direction)
		: Condition(Condition::Type::Turnout, (unsigned char)direction | (turnout & 0b01111111))
	{ }
	inline const Id TurnoutCondition::turnout() const 
	{
		return this->data & 0b01111111;
	}
	inline const TurnoutCondition::Direction TurnoutCondition::direction() const 
	{ 
		return (this->data & 0b10000000) ? Direction::A_C : Direction::A_B;
	}

	class DriveCondition : public Condition
	{
	public:
		enum class Direction : unsigned char
		{
			Forward = 0b0,
			Backward = 0b1
		};
		DriveCondition(const Direction direction);
		const Direction direction() const;
	};
	DriveCondition::DriveCondition(const Direction direction)
		: Condition(Condition::Type::Drive, (unsigned char)direction)
	{ }
	const DriveCondition::Direction DriveCondition::direction() const { return this->data ? Direction::Backward : Direction::Forward; }

	class Connection
	{
	public:
		Connection(const Conditions conditions, const Id target);
		const Conditions conditions;
		const Id target;
		inline const bool isTurnoutConnectionTo(const Id target)
		{
			if (this->target != target)
				return false;
			for (const auto& condition : this->conditions)
				if (condition.type == Condition::Type::Turnout)
					return true;
		}
	};
	using Connections = std::deque<Connection>;

	inline const DriveCondition::Direction directionFromConditions(const Conditions& conditions)
	{
		for (const auto& condition : conditions)
			if (condition.type == Condition::Type::Drive)
				return ((DriveCondition&)condition).direction();
	}

	inline const bool hasTurnoutConditions(const Conditions& conditions)
	{
		for (const auto& condition : conditions)
			if (condition.type == Condition::Type::Turnout)
				return true;

		return false;
	}

	inline const TurnoutCondition::Direction turnoutDirectionFromConditions(const Conditions& conditions)
	{
		for (const auto& condition : conditions)
			if (condition.type == Condition::Type::Turnout)
				return ((TurnoutCondition&)condition).direction();
	}

	inline const TurnoutCondition::Direction other(const TurnoutCondition::Direction& direction)
	{
		return direction == TurnoutCondition::Direction::A_B ? TurnoutCondition::Direction::A_C : TurnoutCondition::Direction::A_B;
	}

	inline Connection::Connection(const Conditions conditions, const Id target)
		: conditions(conditions), target(target)
	{

	}

	class Node
	{
	public:
		Node() = delete;
		Node(const Id id, const Connections connections);
		const Id id;
		const Connections connections;

		inline const Connections otherDirectionConnections(const DriveCondition::Direction than) const
		{
			Connections others;
			for (auto& connection : connections)
				if (directionFromConditions(connection.conditions) != than)
					others.push_back(connection);
			return others;
		}
		inline const Connections sameDirectionConnections(const DriveCondition::Direction like) const
		{
			Connections others;
			for (auto& connection : connections)
				if (directionFromConditions(connection.conditions) == like)
					others.push_back(connection);
			return others;
		}

		const bool isTurnout() const
		{
			for (const auto& connection : connections)
				for (const auto& condition : connection.conditions)
					if (condition.type == Condition::Type::Turnout && ((const TurnoutCondition&)condition).turnout() == id)
						return true;
			return false;
		}

		const bool hasTurnoutConditionTo(const Id that, TurnoutCondition::Direction &direction) const
		{
			for (const auto& connection : connections)
				if(connection.target == that)
					for (const auto &condition : connection.conditions)
						if (condition.type == Condition::Type::Turnout)
						{
							direction = ((const TurnoutCondition&)condition).direction();
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

		/*inline const Connections other(const Id than) const
		{
			Connections others;
			for (auto& connection : connections)
				if (connection.target != than)
					others.push_back(connection);
			return others;
		}

		inline const Conditions to(const Id other) const
		{
			for (auto &c : connections)
				if (c.target == other)
					return c.conditions;

			return {};
		}*/
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
			std::array<TurnoutCondition::Direction, _Nodes> turnouts;
		};

	private:
		struct StepOption
		{
			static bool compare(const StepOption& a, const StepOption& b) {
				return a.state == b.state;
			};

			const size_t id;
			const State state;
			const Conditions conditions;
			
			StepOption(const size_t id, const State state)
				: id(id), state(state), conditions()
			{ };
			StepOption(const size_t id, const State state, const Conditions conditions)
				: id(id), state(state), conditions(conditions)
			{ };
		};
		using StepOptions = std::vector<StepOption>;

		struct Step
		{
			struct Action
			{
				const size_t target;
				const Conditions conditions;
				Action(const size_t target, const Conditions conditions)
					: target(target), conditions(conditions) { };
			};

			const size_t id;
			const State state;
			std::vector<Action> actions;
			
			Step(const size_t id) : id(id), state(), actions() { };
			Step(const size_t id, const State state)
				: id(id), state(state), actions()
			{ };
			Step(const size_t id, const State state, std::vector<Action> actions)
				: id(id), state(state), actions(actions)
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
				t = TurnoutCondition::Direction::A_B;

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

		unsigned int solve(CarPlacement cars)
		{
			State targetState;
			for (auto& s : targetState.slots)
				s = 0;

			unsigned int carIndex = 0;
			for (auto& c : cars)
				targetState.slots[c] = ++carIndex;

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
					if (this->nodes[loco].isTurnout() && hasTurnoutConditions(connection.conditions))
					{
						if (turnoutDirectionFromConditions(connection.conditions) != this->steps[i].state.turnouts[loco])
							continue;
					}
					// check if loco can move
					this->move(i, loco, connection);
				}
			}
			// full graph size
			std::cout << "car placement variants: " << this->steps.size() << "\n";

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
							return (unsigned int)neighbor.conditions.size() + 1;

					return 0;
				});

			typename Dijk::Path shortestPath;
			size_t shortestPathSteps = 0xFFFFFFFF;
			unsigned int solutions = 0;
			for (auto& step : this->steps)
				if (step.state.slots == targetState.slots)
				{
					++solutions;
					auto path = dijkstra.shortestPath(&step);
					if (path.size() < shortestPathSteps)
					{
						shortestPath = path;
						shortestPathSteps = path.size();
					}
				}
			std::cout << "solutions in graph: " << solutions << "\n";

			if (solutions == 0)
				return 0;

			for(auto it = shortestPath.rbegin(); it != shortestPath.rend(); ++it)
				print((unsigned int)(*it)->id, (*it)->state);

			return (unsigned int)shortestPathSteps;
		}

		using Nodes = Nodes<_Nodes>;

	private:
		const Connection outwardConnection(const State& state, const Id id, const DriveCondition::Direction direction, bool& success) const
		{
			if (nodes[id].isTurnout())
			{
				for (const auto& connection : nodes[id].connections)
				{
					if (directionFromConditions(connection.conditions) == direction)
					{
						for (const auto& condition : connection.conditions)
						{
							if (condition.type == Condition::Type::Turnout && ((const TurnoutCondition&)condition).direction() == state.turnouts[id])
							{
								success = true;
								return connection;
							}
						}
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
			return Connection({}, 0);
		}

		void move(const size_t i, const Id from, const Connection connectionTo)
		{
			auto to = connectionTo.target;
			auto conditions = connectionTo.conditions;
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
				auto connection = outwardConnection(state, to, directionFromConditions(conditions), success);
				if (success)
					lastNodedPushedOn = push(state, to, connection);
			}

			pushedSomethingFromTo = pushedSomethingFromTo && state.slots[to] == Empty;

			if (state.slots[to] == Empty)
			{
				if (nodes[to].isTurnout())
				{
					TurnoutCondition::Direction turnoutDirection;
					// from == to == a
					//  other //
					if (nodes[to].hasTurnoutConditionTo(from, turnoutDirection))
					{
						State nextState = state;
						if (!pushedSomethingFromTo || nextState.turnouts[to] == turnoutDirection)
						{
							nextState.turnouts[to] = turnoutDirection;
							auto nextId = nextStep(nextState, from, to, i);
							auto connections = nodes[from].otherDirectionConnections(directionFromConditions(conditions));
							pull(nextId, from, connections, i);

							if (lastNodedPushedOn != from && nodes[lastNodedPushedOn].isTurnout())
							{
								State nextState = state;
								nextState.turnouts[to] = turnoutDirection;
								nextState.turnouts[lastNodedPushedOn] = other(nextState.turnouts[lastNodedPushedOn]);
								auto nextId = nextStep(nextState, from, to, i);
								auto connections = nodes[from].otherDirectionConnections(directionFromConditions(conditions));
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
							for (const auto& condition : connection.conditions)
								if (connection.target != from && condition.type == Condition::Type::Turnout)
								{
									turnoutDirection = ((const TurnoutCondition&)condition).direction();
									State nextState = state;
									if (!pushedSomethingFromTo || nextState.turnouts[to] == turnoutDirection)
									{
										nextState.turnouts[to] = turnoutDirection;
										auto nextId = nextStep(nextState, from, to, i);
										auto connections = nodes[from].otherDirectionConnections(directionFromConditions(conditions));
										pull(nextId, from, connections, i);

										if (lastNodedPushedOn != from && nodes[lastNodedPushedOn].isTurnout())
										{
											State nextState = state;
											nextState.turnouts[to] = turnoutDirection;
											nextState.turnouts[lastNodedPushedOn] = other(nextState.turnouts[lastNodedPushedOn]);
											auto nextId = nextStep(nextState, from, to, i);
											auto connections = nodes[from].otherDirectionConnections(directionFromConditions(conditions));
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
					auto connections = nodes[from].otherDirectionConnections(directionFromConditions(conditions));
					pull(nextId, from, connections, i);

					if (lastNodedPushedOn != from && nodes[lastNodedPushedOn].isTurnout())
					{
						State nextState = state;
						nextState.turnouts[lastNodedPushedOn] = other(nextState.turnouts[lastNodedPushedOn]);
						auto nextId = nextStep(nextState, from, to, i);
						auto connections = nodes[from].otherDirectionConnections(directionFromConditions(conditions));
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
					for (const auto& condition : connection.conditions)
						if (condition.type == Condition::Type::Turnout &&
							((const TurnoutCondition&)condition).direction() == state.turnouts[from])
						{
							auto nextId = nextStep(state, connection.target, targetConnection.target, attach);
							auto connections = nodes[connection.target].sameDirectionConnections(directionFromConditions(connection.conditions));
							pull(nextId, connection.target, connections, attach);
						}
				}
				else
				{
					auto nextId = nextStep(state, connection.target, targetConnection.target, attach);
					auto connections = nodes[connection.target].sameDirectionConnections(directionFromConditions(connection.conditions));
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
			this->steps[attach].addAction(typename Step::Action(nextIndex, {}));

			return nextIndex;			
		}

		const Id push(State& state, const Id from, const Connection connectionTo)
		{
			auto lastNodedPushedOn = from;
			auto to = connectionTo.target;
			auto conditions = connectionTo.conditions;
			if (state.slots[to] == Loco)
				return lastNodedPushedOn;
			else if (state.slots[to] != Empty)
			{
				bool doPush = false;
				if (nodes[to].isTurnout())
				{
					TurnoutCondition::Direction turnoutDirection;
					// from == to == a
					//  other //
					if (nodes[to].hasTurnoutConditionTo(from, turnoutDirection))
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
					auto connection = outwardConnection(state, to, directionFromConditions(conditions), success);
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
			{
				/*bool turnoutsMatch = true;
				for (unsigned int j = 0; j < state.slots.size(); ++j)
				{
					if (this->nodes[j].isTurnout())
						turnoutsMatch = turnoutsMatch && (state.slots[j] == Empty || steps[i].state.turnouts[j] == state.turnouts[j]);
				}*/

				if (steps[i].state.slots == state.slots && steps[i].state.turnouts == state.turnouts)
					return i;
			}
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