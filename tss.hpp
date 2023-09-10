#pragma once
#include <array>
#include <deque>
#include <unordered_set>
#include <vector>
#include <map>
#include <functional>
#include <string>
#include <ios>
#include <iostream>
#include <sstream>

// consider using fixed arrays for nodes and cars, if possible. brings speed x2
#define TSS_WITH_PACKED

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

		static const std::string toString(const TurnoutState t)
		{
			switch (t)
			{
			case TurnoutState::None: return std::string("None");
			case TurnoutState::A_B: return std::string("A_B");
			case TurnoutState::A_C: return std::string("A_C");
			case TurnoutState::DontCare: return std::string("DontCare");
			}

			return "?";
		}

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

		static inline const Connection::TurnoutState other(const Connection::TurnoutState& state)
		{
			if (state == Connection::TurnoutState::None || state == Connection::TurnoutState::DontCare)
				return state;
			return state == Connection::TurnoutState::A_B ? Connection::TurnoutState::A_C : Connection::TurnoutState::A_B;
		}
	};
	using Connections = std::deque<Connection>;

	class Node
	{
	public:
		enum class Options : unsigned char
		{
			CanParkCars = 0b0100000
		};

		static const unsigned char optionMask = (unsigned char)0b11100000;
		static const unsigned char idMask = (unsigned char)~optionMask;

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

	using Nodes = std::vector<Node>;
	inline Node::Node(const Id id, const Connections connections)
		: id(id & Node::idMask), connections(connections), options(id & Node::optionMask)
	{

	}

	class Solver
	{
	public:
		struct State
		{
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

			int findLoco() const
			{
				for (unsigned int i = 0; i < this->slots.size(); ++i)
					if (this->slots[i] == Solver::Loco)
						return i;

				return -1;
			}
		};

		struct PackedStep
		{
			struct State
			{
				using Data = int64_t;
				State(const Solver::State state, const unsigned int turnouts, const unsigned int cars) : turnouts(turnouts), cars(cars), data(State::fromState<true>(state, turnouts, cars)) {};
				State(const Data data) : turnouts(extractTurnouts(data)), cars(extractCars(data)), data(data) {};

				const Connection::TurnoutState turnoutState(const unsigned int turnout) const
				{
					unsigned char t = (this->data >> (6 + cars * 5 + turnout * 2)) & 0b11;
					if (t == 0b00)
						return Connection::TurnoutState::A_B;
					else if (t == 0b01)
						return Connection::TurnoutState::A_C;
					else if (t == 0b10)
						return Connection::TurnoutState::DontCare;
					else
						return Connection::TurnoutState::None;
				}

				const unsigned int node(const unsigned int node) const
				{
					for (unsigned int c = 1; c <= cars; ++c)
						if (((this->data >> ((6 + (c - 1) * 5))) & 0b11111) == (node & 0b11111))
							return c;

					return 0;
				}

				const int64_t data;

				static const unsigned int extractCars(const int64_t data)
				{
					return (data >> 3) & 0b111;
				}

				template<const bool _WithMeta>
				static int64_t fromState(const Solver::State state, const unsigned int turnouts, const unsigned int cars)
				{
					int64_t data = 0;

#define REQ_BITS(n) ceil(log(n) / log(2.0));

					const int64_t metaShift = _WithMeta ? 6 : 0;
					if (_WithMeta)
					{
						data |= (turnouts & 0b111) << 0;
						data |= (cars & 0b111) << 3;
					}
					for (unsigned int c = 1; c <= cars; ++c)
						for (unsigned int i = 0; i < state.slots.size(); ++i)
							if (state.slots[i] == c)
								data |= ((int64_t)(i & 0b11111)) << (int64_t)(metaShift + (c - 1) * 5);

					unsigned int turnout = 0;
					for (unsigned int t = 0; t < state.turnouts.size(); ++t)
						if (state.turnouts[t] != Connection::TurnoutState::None)
						{
							if (state.turnouts[t] == Connection::TurnoutState::A_B)
								data |= ((int64_t)0b00) << (int64_t)(metaShift + cars * 5 + turnout * 2);
							else if (state.turnouts[t] == Connection::TurnoutState::A_C)
								data |= ((int64_t)0b01) << (int64_t)(metaShift + cars * 5 + turnout * 2);
							else if (state.turnouts[t] == Connection::TurnoutState::DontCare)
								data |= ((int64_t)0b10) << (int64_t)(metaShift + cars * 5 + turnout * 2);

							++turnout;
						}

					return data;
				}

			private:
				static const unsigned int extractTurnouts(const int64_t data)
				{
					return data & 0b111;
				}
				const unsigned int turnouts;
				const unsigned int cars;
			};

			struct PackedAction
			{
				const unsigned int parent() const { return _parent; }
				const unsigned int target() const { return data >> 1; }
				const Connection::Direction locoDirection() const { return (data & 0b1) == 0 ? Connection::Direction::Forward : Connection::Direction::Backward; }
				PackedAction(const int64_t data) : _parent(data >> 32), data(data & 0xFFFFFFFF) { };
				PackedAction(const unsigned int target, const Connection::Direction locoDirection)
					: _parent(0)
					, data(target << 1 | ((locoDirection == Connection::Direction::Forward) ? 0 : 1)) { };

				const unsigned int _parent;
				const unsigned int data;
			};
			//PackedStep(const PackedStep & step) : state{ step.state }, actions{ step.actions } {};

			PackedStep(const PackedStep::State state, std::vector<PackedAction> actions) 
				: state{ state }
				, actions{ (PackedAction*)malloc(actions.size() * sizeof(PackedAction))}, actionsCount{actions.size()} 
			{
				memcpy(this->actions, actions.data(), actions.size() * sizeof(PackedAction));
			};
			/*
			PackedStep operator=(const PackedStep &other)
			{
				PackedStep s(other);
				return s;
			}*/

			PackedStep(const State::Data state) : state{ state }, actions(), actionsCount(0) { };

			void setActions(PackedAction * actions)
			{
				this->actions = actions;
			}

			void incActionsCount()
			{
				this->actionsCount++;
				//actions.emplace_back(std::forward<const unsigned int>(data));
			}

			const State state;
			// results in vc internal compiler error: std::vector<PackedAction> actions;
			PackedAction * actions;
			size_t actionsCount;
		};
		
		class Precomputed
		{
		public:
			struct Action
			{
				Action(const int64_t data) : data{ data } { };
				Action(const unsigned int id, const unsigned int data) : data{ (((int64_t)id) << 32) | data } {};
				const unsigned int id() const { return (unsigned int)(data >> 32); };
				const unsigned int content() const { return (unsigned int)(data & 0xFFFFFFFF); };
				const int64_t data;
			};
			/*
			struct Step
			{
				const int64_t state;
			};*/

			struct Storage
			{
				const unsigned int carsCount;
				PackedStep * const steps;
				const unsigned int stepsCount;
				PackedStep::PackedAction * actions;
				const unsigned int actionsCount;
			};
		};
		using PackedSteps = std::vector<PackedStep>;

		struct Step
		{
			struct Action
			{
				const unsigned int target;
				const Connection::Direction locoDirection;
				Action(const unsigned int target, const Connection::Direction locoDirection)
					: target(target), locoDirection(locoDirection) { };
			};

			const unsigned int id;
			const State state;
			std::vector<Action> actions;
			
			Step(const unsigned int id) : id(id), state(), actions() { };
			Step(const unsigned int id, const State state) : id(id), state(state), actions() { };
			Step(const unsigned int id, const State state, std::vector<Action> actions) : id(id), state(state), actions(actions) { };

			Step operator=(Step other)
			{
				Step s(other);
				return s;
			}

			Step(const Step& s) : id(s.id), state(s.state), actions(s.actions) { }

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
		using Dijk = Dijkstra<unsigned long>;
		using Steps = std::vector<Step>;

	public:
		enum class Result : unsigned int
		{
			OK,
			NoLoco,
			Deadlock,
			IncompatibleTypes
		};

		using CarPlacement = std::vector<unsigned int>;

		using PrintCallback = std::function<void(const std::string, const PackedStep::State&)>;
		using GraphCreationCallback = std::function<void(const unsigned int step, const unsigned int steps, const unsigned int solutions)>;
		using StatisticsCallback = std::function<void(const unsigned int steps, const unsigned int solutions)>;

		Solver(const Nodes nodes, PrintCallback print, GraphCreationCallback creation, StatisticsCallback statistics)
			: nodes(nodes), print(print), creation(creation), statistics(statistics), dijkstra(0xFFFFFFFF)
		{

		}

		Result init(Solver::Precomputed::Storage storage)
		{
			if (this->packedSteps != nullptr)
				delete this->packedSteps;

			this->packedStepsSize = storage.stepsCount;
			this->packedSteps = storage.steps; //(PackedStep*)malloc(sizeof(PackedStep) * this->packedStepsSize);
			this->countTurnouts();

			//unsigned int turnouts = countTurnouts(), cars = countCars();
			
			size_t lastActionStepId = (size_t)-1;
			for (unsigned int a = 0; a < storage.actionsCount; ++a)
			{
				if (storage.actions[a].parent() != lastActionStepId)
				{
					lastActionStepId = storage.actions[a].parent();
					this->packedSteps[storage.actions[a].parent()].setActions(&storage.actions[a]);
				}
				this->packedSteps[storage.actions[a].parent()].incActionsCount();
			}
			/*;
			unsigned int a = 0;
			for (unsigned int i = 0; i < this->packedStepsSize; ++i)
			{
				
				const auto& step = storage.steps[i];
				const PackedStep::State state(step.state);

				std::vector<PackedStep::PackedAction> actions;
				// we stored them in order
				while (storage.actions[a].id() == i)
				{
					actions.emplace_back(storage.actions[a].content());
					++a;
				}

				new (packedSteps + i) PackedStep(state, actions);
				
			}*/

			return Result::OK;
		}

		Result init(CarPlacement cars, const bool keep = false)
		{
			State state;
			state.slots.resize(this->nodes.size());
			state.turnouts.resize(this->nodes.size());

			stateMap.clear();
			numTurnouts = 0;
			numCars = (unsigned int)cars.size();

			for (unsigned int i = 0; i < this->nodes.size(); ++i)
			{
				state.turnouts[i] = this->nodes[i].isTurnout() ? Connection::TurnoutState::A_B : Connection::TurnoutState::None;
				numTurnouts += this->nodes[i].isTurnout() ? 1 : 0;
			}

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
			this->nextStepIndex = (unsigned int)this->steps.size();

			int nextIndex = this->hasStepWithState(state);
			if (nextIndex == -1)
				nextIndex = (int)this->addStep(state);

			return Result::OK;
		}

		const CarPlacement random(unsigned int _Cars) 
		{
			CarPlacement result(_Cars);

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

		const CarPlacement fromStepsGraph(const unsigned int i) const
		{
			CarPlacement result;
			const unsigned int cars = countCars();
			result.resize(cars);
			for (unsigned int j = 0; j < this->nodes.size(); ++j)
				for (unsigned int k = 1; k <= cars; ++k)
					if (this->steps[i].state.slots[j] == k)
						result[k - 1] = j;

			return result;
		}

		const CarPlacement fromPackedStepsGraph(const unsigned int i) const
		{
			CarPlacement result;
			const unsigned int cars = countCars();
			result.resize(cars);
			for (unsigned int j = 0; j < this->nodes.size(); ++j)
				for (unsigned int k = 1; k <= cars; ++k)
					if (this->packedSteps[i].state.node(j) == k)
						result[k - 1] = j;

			return result;
		}

		const unsigned int getNumTurnouts() const
		{
			return this->numTurnouts;
		}

		const Connection::TurnoutState turnoutState(const unsigned int step, const unsigned int turnout) const
		{
			if (step > this->packedStepsSize || turnout > this->getNumTurnouts())
				return Connection::TurnoutState::None;

			return this->packedSteps[step].state.turnoutState(turnout);
		}

		const unsigned int stepsCount() const
		{
			return this->packedStepsSize;
		}

		const unsigned int randomEndState(unsigned int start, unsigned int difficulty = -1) const
		{
			if(difficulty == -1)
				return (start + std::rand()) % this->stepsCount();
			else
			{
				const unsigned int base = 20;
				const unsigned int dev = 4;
				auto range = (base + dev) * (difficulty+1);
				auto random = (std::rand() % dev + base) * (difficulty+1);
				if (start < range / 2)
					return (start + range) % this->stepsCount();
				else if (start + range / 2 > this->stepsCount())
					return (start - range / 2) % this->stepsCount();
				else
					return start - range / 2 + random;
			}
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

			creation((unsigned int)i, (unsigned int)this->steps.size(), 0);
			this->nextStepIndex += 1;

			return this->nextStepIndex < this->steps.size();
		}

		void solve_dijkstra_init(const unsigned int start = 0)
		{
			dijkstra.dijkstra_init(this->stepsCount(), start);
		}

		const unsigned int solve_dijkstra_expectedIterations()
		{
			return this->stepsCount();
		}

		void solve_dijkstra_markEndStepsLike(const unsigned int selectedEndStep)
		{
			this->endStates.clear();
			for (unsigned int i = 0; i < this->stepsCount(); ++i)
			{
				unsigned int matching = 0;
				for (unsigned int n = 0; n < this->nodes.size(); ++n)
					matching += (this->packedSteps[i].state.node(n) == this->packedSteps[selectedEndStep].state.node(n)) ? 1 : 0;

				if (matching == this->nodes.size())
					this->endStates.push_back(i);
			}
		}

		const unsigned int solve_dijkstra_markEndStepsLike(const CarPlacement carPlacementEnd)
		{
			this->endStates.clear();
			for (unsigned int i = 0; i < this->stepsCount(); ++i)
			{
				unsigned int matching = 0;
				for (unsigned int c = 0; c < carPlacementEnd.size(); ++c)
					matching += (this->steps[i].state.slots[carPlacementEnd[c]] == c + 1) ? 1 : 0;

				if (matching == carPlacementEnd.size())
					this->endStates.push_back(i);
			}

			return (unsigned int)this->endStates.size();
		}

		struct Cost
		{
			const unsigned int cost;
			const Connection::Direction locoDirection;
			Cost(const unsigned int cost, const Connection::Direction locoDirection)
				: cost(cost), locoDirection(locoDirection)
			{

			}
		};

		const Cost costAndDirection(size_t previous, size_t from, size_t to)
		{
			unsigned int cost = 0;
			Connection::Direction locoDirection = Connection::Direction::Forward;
			//for (auto neighbor : this->packedSteps[from].actions)
			for (size_t a = 0; a < this->packedSteps[from].actionsCount; ++a)
			{
				const auto neighbor = this->packedSteps[from].actions[a];
				if (neighbor.target() == to)
				{
					if (previous != (size_t)-1)
					{
						locoDirection = neighbor.locoDirection();
						// + direction changed ? 1 : 0
						size_t previousToHere = -1;
						for (size_t p = 0; p < this->packedSteps[previous].actionsCount; ++p)
						{
							if (this->packedSteps[previous].actions[p].target() == from)
							{
								previousToHere = p;
								break;
							}
						}
						if (previousToHere != -1)
							cost += (this->packedSteps[previous].actions[previousToHere].locoDirection() != neighbor.locoDirection() ? 1 : 0);

						/*
						const auto it = std::find_if(this->packedSteps[previous].actions.begin(), this->packedSteps[previous].actions.end(),
							[&from](const PackedStep::PackedAction& action) -> bool {
								return action.target() == from;
							});
						if (it != this->packedSteps[previous].actions.end())
							cost += ((*it).locoDirection() != neighbor.locoDirection() ? 1 : 0);
							*/
						// + #turnouts changed
						for (unsigned int t = 0; t < this->getNumTurnouts(); ++t)
							cost += this->packedSteps[from].state.turnoutState(t) != this->packedSteps[to].state.turnoutState(t) ? 1 : 0;

						// moved cars in contrast to prev
					}
					return Cost(cost, locoDirection);
				}
			}
			return Cost(cost, locoDirection);
		}

		unsigned int cost(size_t previous, size_t from, size_t to)
		{
			return costAndDirection(previous, from, to).cost;
		}

		const bool solve_dijkstra_step()
		{
			return solve_dijkstra_step<1>();
		}

		template<unsigned int _Iterations>
		const bool solve_dijkstra_step()
		{
			unsigned int it = 0;
			while (it < _Iterations)
			{
				++it;
				if (!dijkstra.dijkstra_step([&](const size_t i) -> const std::vector<size_t> {
					std::vector<size_t> neighbors;
					//for (auto neighbor : this->packedSteps[i].actions)
					for (size_t a = 0; a < this->packedSteps[i].actionsCount; ++a)
					{
						auto neighbor = this->packedSteps[i].actions[a];
						neighbors.push_back(neighbor.target());
					}

					return neighbors;
					},
					[&](const size_t a, const size_t b, Dijk::PreviousCallback prec) -> const unsigned long {
						const auto preA = prec(a);
						if (preA != (size_t)-1)
							return this->cost(preA, a, b);
						return 0;
					}))
					return false;
			}
			return true;/*/
		}
		
			return dijkstra.dijkstra_step([&](const size_t i) -> const std::vector<size_t> {
				std::vector<size_t> neighbors;
#ifdef TSS_WITH_PACKED
				for (auto neighbor : this->packedSteps[i].actions)
					neighbors.push_back(neighbor.target());
#else
				for (auto neighbor : this->steps[i].actions)
					neighbors.push_back(neighbor.target);
#endif
				return neighbors;
				},
				[&](const size_t a, const size_t b, Dijk::PrecStorage::GetCallback prec) -> const unsigned long {
					const auto preA = prec(a);
					if (preA != Dijk::unset)
						return this->cost(preA, a, b);
					return 0;
					/*
					// look through prec of a to decide how man turnouts changed
#ifdef TSS_WITH_PACKED
					for (auto neighbor : this->packedSteps[a].actions)
						if (neighbor.target() == b)
#else
					for (auto neighbor : this->steps[a].actions)
						if (neighbor.target == b)
#endif							
						{
							const auto preA = prec(a);
							if (preA != Dijk::unset)
								return this->cost(preA, a, b);
							
							{
#ifdef TSS_WITH_PACKED
								// + direction changed ? 1 : 0
								const auto it = std::find_if(this->packedSteps[preA].actions.begin(), this->packedSteps[preA].actions.end(),
									[&a](const PackedStep::PackedAction& action) -> bool {
										return action.target() == a;
									});
								if (it != this->packedSteps[preA].actions.end())
									cost += ((*it).locoDirection() != neighbor.locoDirection() ? 1 : 0);
								// + #turnouts changed
								for (unsigned int t = 0; t < this->getNumTurnouts(); ++t)
									cost += this->packedSteps[a].state.turnoutState(t) != this->packedSteps[b].state.turnoutState(t) ? 1 : 0;
#else
								// + direction changed ? 1 : 0
								const auto it = std::find_if(this->steps[preA].actions.begin(), this->steps[preA].actions.end(),
									[&a](const Step::Action& action) -> bool {
										return action.target == a;
									});
								if (it != this->steps[preA].actions.end())
									cost += ((*it).locoDirection != neighbor.locoDirection ? 1 : 0);
								// + #turnouts changed
								for (unsigned int t = 0; t < this->getNumTurnouts(); ++t)
									cost += this->steps[a].state.turnouts[t] != this->steps[b].state.turnouts[t] ? 1 : 0;
#endif	
							}
							 
							return cost;
						}

					return 0;
				});*/
		}

		typename Dijk::Path solve_dijkstra_shortestPath()
		{
			typename Dijk::Path shortestPath;
			unsigned int currentSolutionCheck = 0;
			unsigned int shortestPathSteps = dijkstra.infinity;
			for (const unsigned int i : this->endStates)
			{
				//auto& step = this->packedSteps[i];
				++currentSolutionCheck;
				auto path = dijkstra.shortestPath(i);
				if (path.size() < shortestPathSteps)
				{
					shortestPath = path;
					shortestPathSteps = (unsigned int)path.size();
				}
			}
			statistics((unsigned int)this->stepsCount(), (unsigned int)this->endStates.size());
			if (this->endStates.size() == 0)
				return {};

			print("Start\n", this->packedSteps[0].state);
			PackedStep::State packedTargetState(targetState, this->getNumTurnouts(), countCars());
			print("End\n", packedTargetState);

#ifdef _DEBUG
			for (auto it = shortestPath.rbegin(); it != shortestPath.rend(); ++it)
				print(std::string("Step").append(std::to_string((unsigned int)(*it))), this->packedSteps[(*it)].state);

#endif
			return shortestPath;
		}

		unsigned int solve(const CarPlacement cars, const bool maySelectRandomIfNoneFound)
		{
			createGraph();
			markEndSteps(cars, maySelectRandomIfNoneFound);
			return quickSolve();
		}

		void createGraph()
		{
			if (this->nextStepIndex == 0)
			{
				// build graph
				while (solve_step());
				pack();
			}
		}

		const unsigned int quickSolve()
		{
			if (this->stepsCount() == 0)
				return 0;

			solve_dijkstra_init();

			while (solve_dijkstra_step());

			return (const unsigned int)solve_dijkstra_shortestPath().size();
		}

		const unsigned int countTurnouts()
		{
			if (numTurnouts != -1)
				return numTurnouts;

			numTurnouts = 0;
			for (const auto& node : this->nodes)
				if (node.isTurnout())
					++numTurnouts;

			return numTurnouts;
		}

		const unsigned int countCars() const
		{
			unsigned int cars = 0;
			if (this->steps.size() > 0)
			{
				for (unsigned int i = 0; i < this->steps[0].state.slots.size(); ++i)
					if (this->steps[0].state.slots[i] != Empty)
						++cars;

			}
			else if (this->packedStepsSize > 0)
			{
				cars = PackedStep::State::extractCars(this->packedSteps[0].state.data);
			}
			return cars;
		}

		void markEndSteps(const CarPlacement cars, const bool maySelectRandomIfNoneFound)
		{
			State targetState;
			targetState.slots.clear();
			targetState.turnouts.clear();
			targetState.slots.resize(this->nodes.size());
			targetState.turnouts.resize(this->nodes.size());

			for (auto& s : targetState.slots)
				s = 0;

			unsigned int carIndex = 0;
			for (auto& c : cars)
				targetState.slots[c] = ++carIndex;

			this->endStates.clear();
			for (unsigned int i = 0; i < this->stepsCount(); ++i)
			{
				unsigned int matching = 0;
				for (unsigned int n = 0; n < this->nodes.size(); ++n)
					matching += (this->packedSteps[i].state.node(n) == targetState.slots[n]) ? 1 : 0;

				if(matching == this->nodes.size())
					this->endStates.push_back(i);
			}
		}

		void pack()
		{
			if (this->packedSteps != nullptr)
				delete this->packedSteps;
			this->packedSteps = (PackedStep*)malloc(sizeof(PackedStep) * this->steps.size());
			this->packedStepsSize = (unsigned int)this->steps.size();

			unsigned int turnouts = this->getNumTurnouts(), cars = countCars();

			for (unsigned int i = 0; i < this->steps.size(); ++i)
			{
				const auto& step = this->steps[i];
				const PackedStep::State state(step.state, turnouts, cars);

				std::vector<PackedStep::PackedAction> actions;
				for (const auto& action : step.actions)
					actions.emplace_back((unsigned int)action.target, action.locoDirection);

				new (packedSteps + i) PackedStep(state, actions);
			}
		}

#ifdef TSS_WITH_EXPORT
		void exportSteps(std::ostream& cpp, std::ostream& hpp, std::string hppName, std::string name, std::string define)
		{
			/*
				#define define
				#include "../tss.hpp"
				static const unsigned int tss_steps_classic_2_cars = 2;
				static const unsigned int tss_steps_classic_2_size = 505;
				static TimeSaver::Solver::Precomputed::Step tss_steps_classic_2[] = { 
				 0x2855,
				 0x2aa2815,
				 };
				
				static const unsigned int tss_steps_classic_2_actions_size = 1061;
				static TimeSaver::Solver::Precomputed::Action tss_steps_classic_2_actions[] = { 
				0x3,
				0x4,
				0x100000006,
				0x200000007,
				};
			*/
			hpp << "#define " << define << "\n";
			hpp << "#include \"../tss.hpp\"\n";
			hpp << "extern const unsigned int " << name << "_cars;\n";
			hpp << "extern const unsigned int " << name << "_steps_size;\n";
			hpp << "extern TimeSaver::Solver::PackedStep " << name << "_steps[];\n";
			hpp << "extern const unsigned int " << name << "_actions_size;\n";
			hpp << "extern TimeSaver::Solver::PackedStep::PackedAction " << name << "_actions[];\n";

			cpp << "#include \"" << hppName << "\"\n";
			cpp << "const unsigned int " << name << "_cars = " << std::dec << this->countCars() << ";\n";
			cpp << "const unsigned int " << name << "_steps_size = " << std::dec << this->stepsCount() << ";\n";
			cpp << "TimeSaver::Solver::PackedStep " << name << "_steps[] = { \n";

			unsigned int actionsCount = 0;
			for (unsigned int i = 0; i < this->stepsCount(); ++i)
			{
				const auto& step = this->packedSteps[i];
				// State
				cpp << "0x" << std::hex << (int64_t)step.state.data << ",\n";
				actionsCount += (unsigned int)step.actionsCount;
			}
			cpp << "};\n";
			cpp << "const unsigned int " << name << "_actions_size = " << std::dec << actionsCount << ";\n";
			cpp << "TimeSaver::Solver::PackedStep::PackedAction " << name << "_actions[] = { \n";
			// put actions into one array, re-alignmend done on loading
			for (unsigned int i = 0; i < this->stepsCount(); ++i)
			{
				const auto& step = this->packedSteps[i];
				for (unsigned int a = 0; a < step.actionsCount; ++a)
				{
					const auto& action = step.actions[a];
					cpp << "0x" << std::hex << (int64_t)(((int64_t)i << 32) | (int64_t)action.data) << ",\n";
				}
			}
			cpp << "};\n";
		}
#endif

	private:

		const Connection outwardConnection(const State& state, const Id id, const Connection::Direction direction, bool& success) const
		{
			if (nodes[id].isTurnout())
			{
				for (const auto& connection : nodes[id].connections)
				{
					if (connection.direction == direction && 
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

		void move(const unsigned int i, const Id from, const Connection connectionTo)
		{
			auto to = connectionTo.target;
			const Connection::Direction locoDirection = connectionTo.direction;
			//auto conditions = connectionTo.conditions;
			int lastNodePushedOn = from;

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
				pushedStates = {{ { lastNodePushedOn, state } }};

			for (const auto& ps : pushedStates)
			{
				const auto lastNodePushedOn = ps.lastNodePushedOn;
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
							if (!pushedSomethingFromTo || state.turnouts[to] == turnoutDirection || state.turnouts[to] == Connection::TurnoutState::DontCare)
							{
								pull<true>(state, i, from, connectionTo, lastNodePushedOn, turnoutDirection);
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
									if (!pushedSomethingFromTo || state.turnouts[to] == turnoutDirection || state.turnouts[to] == Connection::TurnoutState::DontCare)
									{
										pull<true>(state, i, from, connectionTo, lastNodePushedOn, turnoutDirection);
									}
									
								}
							}
						}
					}
					else
						// to == from
					{
						pull<false>(state, i, from, connectionTo, lastNodePushedOn);
					}
				}
			}
		}

		template<bool _setTurnoutTo>
		void pull(const State &state, const unsigned int i, const Id from, const Connection connectionTo, const int lastNodePushedOn, Connection::TurnoutState turnoutDirection = Connection::TurnoutState::None)
		{
			auto to = connectionTo.target;
			const Connection::Direction locoDirection = connectionTo.direction;
			State nextState = state;
				if(_setTurnoutTo)
					nextState.turnouts[to] = turnoutDirection;
				auto updatedStepState = nextStep(nextState, from, to, i, locoDirection);
				auto connections = nodes[from].otherDirectionConnections(connectionTo.direction);
				pull(updatedStepState, from, connections, i, locoDirection);

				// we pushed the last car onto a turnout, so add the toggled pushedOnTurnout as well
				if (lastNodePushedOn != -1 && lastNodePushedOn != from && nodes[lastNodePushedOn].isTurnout())
				{
					State nextState = state;
					if (_setTurnoutTo)
						nextState.turnouts[to] = turnoutDirection;
					nextState.turnouts[lastNodePushedOn] = Connection::other(nextState.turnouts[lastNodePushedOn]);
					auto updatedStepState = nextStep(nextState, from, to, i, locoDirection);
					auto connections = nodes[from].otherDirectionConnections(connectionTo.direction);
					pull(updatedStepState, from, connections, i, locoDirection);
				}
		}

		// pull cars onto from
		void pull(State fromState, const Id from, const Connections connections, const unsigned int attach, const Connection::Direction locoDirection)
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

		State nextStep(State state, const Id from, const Id to, const unsigned int attach, const Connection::Direction locoDirection)
		{
			state.slots[to] = state.slots[from];
			state.slots[from] = Empty;

			if (checkValidState(state))
			{
				auto putState = state;
				updateDontCare(putState);

				int nextIndex;
//#pragma omp critical
				{
				nextIndex = this->hasStepWithState(putState);
				if (nextIndex == -1)
					nextIndex = (int)this->addStep(putState);
				}
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
			int lastNodePushedOn = -1;
			auto to = connectionTo.target;

			if (state.slots[to] == Loco)
				return {{ {lastNodePushedOn, state} }};

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
					// else there is something on the slot, so the turnout is set

					if (state.turnouts[to] == turnoutDirection)
					{
						state.turnouts[to] = turnoutDirection;
						doPush = true;
					}
					lastNodePushedOn = -1;

					thisVals.emplace_back(lastNodePushedOn, state);
				}
				else
					// otherA == to == from
					//   otherB //
				{
					doPush = true;
					lastNodePushedOn = to;
					if (state.turnouts[to] == Connection::TurnoutState::DontCare)
					{
						{
							auto newState = state;
							newState.turnouts[to] = Connection::TurnoutState::A_B;
							thisVals.emplace_back(lastNodePushedOn, newState);
						}
						{
							auto newState = state;
							newState.turnouts[to] = Connection::TurnoutState::A_C;
							thisVals.emplace_back(lastNodePushedOn, newState);
						}
					}
					else
					{
						thisVals.emplace_back(lastNodePushedOn, state);
					}
				}
			}
			else
			{
				doPush = true;
				thisVals.emplace_back(lastNodePushedOn, state);
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
								state.slots[to] = state.slots[from];
								state.slots[from] = Empty;
							}
							retVals.emplace_back(nps.lastNodePushedOn, state);
						}
					}
				}
				else if (state.slots[to] == Empty)
				{
					state.slots[to] = state.slots[from];
					state.slots[from] = Empty;
					retVals.emplace_back(ps.lastNodePushedOn, state);
				}
				else
					retVals.emplace_back(ps.lastNodePushedOn, state);
			}
			return retVals;
		}

		template<typename... Args>
		const unsigned int addStep(Args&&... args)
		{
			unsigned int id = (unsigned int)this->steps.size();
			this->steps.emplace_back(id, std::forward<Args>(args)...);

			auto key = PackedStep::State::fromState<false>(this->steps[id].state, numTurnouts, numCars);
			this->stateMap.emplace(key, id);

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
			auto key = PackedStep::State::fromState<false>(state, numTurnouts, numCars);
			auto it = this->stateMap.find(key);
			int i = it == this->stateMap.end() ? -1 : (int)it->second;

			return i;
		}

		int findLoco(const State& state) const
		{
			return state.findLoco();
		}

		static const unsigned int Loco = 1;
		static const unsigned int Empty = 0;
		unsigned int loco = 0;

		unsigned int nextStepIndex = 0;
		State targetState;
		Dijk dijkstra;

		const Nodes nodes;
		PrintCallback print;
		GraphCreationCallback creation;
		StatisticsCallback statistics;

		Steps steps;
		std::map<uint64_t, size_t> stateMap;
		unsigned int numCars;
		unsigned int numTurnouts = -1;

		PackedStep *packedSteps = nullptr;
		unsigned int packedStepsSize = 0;

		std::vector<unsigned int> endStates;
	};
}
