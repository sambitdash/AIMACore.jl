export  Problem,
            result, step_cost, goal_test, actions, heuristic, state_value, successor_states,
        Node,
            solution, failure, isless,
        SearchAlgorithm,
            BreadthFirstSearch,
                execute,
            UniformCostSearch,
            DepthLimitedSearch,
            IterativeDeepeningSearch,
            RecursiveBestFirstSearch,
            GraphSearch,
                GraphSearchDepth,
                GraphSearchBreadth,
                GraphSearchUniformCost,
                GraphSearchBestFirst,
                GraphSearchAStar,
            HillClimbingSearch,
            SimulatedAnnealingSearch,
        AIMASearchFIFO,
        AIMASearchLIFO,
        AIMASearchPQ,
        AIMASearchSet

using Compat

using DataStructures

import Base: ==, isless, in, eltype, length, empty!

@compat abstract type Problem end

result(problem::Problem, state::State, action::Action) = error(E_ABSTRACT)
step_cost(problem::Problem, state::State, action::Action) = error(E_ABSTRACT)
goal_test(problem::Problem, state::State) = error(E_ABSTRACT)
actions(problem::Problem,state::State) = error(E_ABSTRACT)
heuristic(problem::Problem, state::State) = error(E_ABSTRACT)
state_value(problem::Problem, state::State) = error(E_ABSTRACT)
successor_states(problem::Problem, state::State) = error(E_ABSTRACT)


search(problem::Problem) = execute(problem.search_algorithm, problem)

mutable struct NodeT{S<:State, T}
    state::S
    parent::Nullable{NodeT{S, T}}
    action::Action
    path_cost::Number
    f::Number
end

const Node{S <: State} = NodeT{S, :none}

NodeT(state::S, s::Symbol=:none,
      action::Action=Action_NoOp, path_cost::Number=0, f::Number=0) where {S<: State} =
    NodeT{S, s}(state, Nullable{NodeT{S, s}}(), action, path_cost, f)

NodeT(state::S, parent::NodeT{S,T},
    action::Action=Action_NoOp,
    path_cost::Number=zero(parent.path_cost),
    f::Number=zero(parent.f)) where {S <:State, T} =
    NodeT{S, T}(state, Nullable(parent), action, path_cost, f)

isless(n1::NodeT, n2::NodeT) = isless(n1.f, n2.f)

make_node{S<:State}(state::S, s::Symbol) = NodeT(state, s)
make_node{S<:State}(state::S) = make_node(state, :none)

function child_node{S <:State}(problem, parent::NodeT{S, :none}, action)
    state = result(problem, parent.state, action)
    path_cost = parent.path_cost + step_cost(problem, parent.state, action)
    return NodeT(state, parent, action, path_cost, path_cost)
end

function child_node{S <:State}(problem, parent::NodeT{S, :greedy}, action)
    state = result(problem, parent.state, action)
    path_cost = parent.path_cost + step_cost(problem, parent.state, action)
    return NodeT(state, parent, action, path_cost, heuristic(problem, state))
end

function child_node{S <:State}(problem, parent::NodeT{S, :astar}, action)
    state = result(problem, parent.state, action)
    path_cost = parent.path_cost + step_cost(problem, parent.state, action)
    return NodeT(state, parent, action, path_cost, path_cost + heuristic(problem, state))
end

"""
The default implementation of *solution* is returning the sequence of nodes.
This may not be ideal for many implementations. They may provide a more
elaborate reporting as well.
"""
function solution{S<:State, T}(node::NodeT{S, T})
    nodes = Vector{NodeT{S, T}}()
    while true
        unshift!(nodes, node)
        isnull(node.parent) && break
        node = get(node.parent)
    end
    return nodes
end

"""
The default implementation of *failure* is throwing an error
"""
failure(node::NodeT) = error(E_NODE_NOT_REACHABLE)

@compat abstract type SearchAlgorithm end

const AIMASearchSet{S<:State} = Set{S}

mutable struct AIMASearchQueue{S<:State, T}
     s::Vector{Node{S}}
     AIMASearchQueue{S, T}() where {S<:State, T} = new(Vector{Node{S}}())
end
eltype(q::AIMASearchQueue) = eltype(q.s)
empty(q::AIMASearchQueue) = empty!(q.s)

const AIMASearchFIFO{S<:State} = AIMASearchQueue{S, :fifo}
const AIMASearchLIFO{S<:State} = AIMASearchQueue{S, :lifo}


isempty(q::AIMASearchQueue) = isempty(q.s)
insert{S<:State, T}(q::AIMASearchQueue{S, T}, d::Node{S}) = push!(q.s, d)
length(q::AIMASearchQueue) = length(q.s)

pop(q::AIMASearchFIFO) = shift!(q.s)
pop(q::AIMASearchLIFO) = pop!(q.s)

function insert_or_replace{S <: State, T}(f::Function,
    q::AIMASearchQueue{S, T}, d::Node{S})
    state = start(q.s)
    while !done(q.s, state)
        ps = state
        (i, state) = next(q.s, state)
        i.state == d.state && return f(i) ? splice!(q.s, ps, [d]) : nothing
    end
    return insert(q.s, d)
end

in{S <: State, T}(state::S, q::AIMASearchQueue{S, T}) = any(x -> x.state == state, q.s)

"""
*BreadthFirstSearch* An uninformed graph search technique to reach goal.

Cost of search is completely ignored.

*frontier* is a FIFO
*explored* is a Set

pg. 82 Fig. 3.11 AIMA 3ed
"""
mutable struct BreadthFirstSearch{FIFO, SET, S<:State} <: SearchAlgorithm
    frontier::FIFO
    explored::SET
    BreadthFirstSearch{FIFO, SET, S}() where{FIFO, SET, S<:State} =
        new(has_trait_queue(FIFO{S}()), has_trait_set(SET{S}()))
end

BreadthFirstSearch{S<:State}(::S) = BreadthFirstSearch{AIMASearchFIFO, AIMASearchSet, S}()

function execute(search::BreadthFirstSearch, problem::Problem)
    empty(search.frontier)
    empty(search.explored)
    node = NodeT(problem.initial_state, :none)
    goal_test(problem, node.state) && return solution(node)
    insert(search.frontier, node)

    while(true)
        isempty(search.frontier) && return failure(node)
        node = pop(search.frontier)
        append(search.explored, node.state)

        for action in actions(problem, node.state)
            child = child_node(problem, node, action)
            !(child.state in search.explored) && !(child.state in search.frontier) &&
            goal_test(problem, child.state) && return solution(child)
            insert(search.frontier, child)
        end
    end
end

const AIMASearchPQ_Base{S<:State, NT} = PriorityQueue{S, NodeT{S, NT}}

pop(pq::AIMASearchPQ_Base) = ((key,val) = peek(pq); dequeue!(pq); val)
peekv(pq::AIMASearchPQ_Base) = ((key,val) = peek(pq); val)
empty(pq::AIMASearchPQ_Base) = while(!isempty(pq));dequeue!(pq);end
insert{S <: State, NT}(pq::AIMASearchPQ_Base{S, NT}, node::NodeT{S, NT}) =
    enqueue!(pq, node.state, node)
replace{S <: State, NT}(pq::AIMASearchPQ_Base{S, NT}, node::NodeT{S, NT}) =
    (pq[node.state] = node)
in{S <: State, NT}(state::S, pq::AIMASearchPQ_Base{S, NT}) = haskey(pq, state)

function insert_or_replace{S <: State, NT}(f::Function,
    pq::AIMASearchPQ_Base{S, NT}, node::NodeT{S, NT})
    !haskey(pq, node.state) && return enqueue!(pq, node.state, node)
    f(pq[node.state]) && return (pq[node.state] = node)
end

const AIMASearchPQ{S <: State}   = AIMASearchPQ_Base{S, :none}
const AIMASearchPQ_G{S <: State} = AIMASearchPQ_Base{S, :greedy}
const AIMASearchPQ_A{S <: State} = AIMASearchPQ_Base{S, :astar}

"""
*UniformCostSearch* An uninformed graph search technique to reach goal.

Cost of search is used in taking decisions.

*frontier* is a priority queue
*explored* is a Set

pg. 82 Fig. 3.11 AIMA 3ed
"""
mutable struct UniformCostSearch{PQ, SET, S<:State} <: SearchAlgorithm
    frontier::PQ
    explored::SET
    UniformCostSearch{PQ, SET, S}() where {PQ, SET, S<:State} =
        has_trait_queue(PQ{S}, Node{S}) &&
        method_exists(replace, (PQ{S}, Node{S})) &&
        method_exists(in, (S, PQ{S})) &&
        method_exists(get, (PQ{S}, S, Node{S})) &&
        new(PQ{S}(), has_trait_set(SET{S}()))
end

UniformCostSearch{S<:State}(::S) = UniformCostSearch{AIMASearchPQ, AIMASearchSet, S}()

function execute(search::UniformCostSearch, problem) #returns a solution, or failure
    empty(search.frontier)
    empty(search.explored)
    node = make_node(problem.initial_state)
    insert(search.frontier, node)

    while(true)
        isempty(search.frontier) && return failure(node)
        node = pop(search.frontier)
        goal_test(problem, node.state) && return solution(node)
        append(search.explored, node.state)

        for action in actions(problem, node.state)
            child = child_node(problem, node, action)
            if !(child.state in search.explored) && !(child.state in search.frontier)
                insert(search.frontier, child)
            elseif (child.state in search.frontier) &&
                    (child.path_cost < get(search.frontier, child.state,
                                           make_node(problem.initial_state)).path_cost)
                replace(search.frontier, child)
            end
        end
    end
end

"""
*DepthLimitedSearch* An uninformed graph search technique to reach goal.

This is a depth first search limited by the depth of the search.

*limit* depth to which search should be conducted.

pg. 88 Fig. 3.17 AIMA 3ed
"""
struct DepthLimitedSearch <: SearchAlgorithm
    limit::Int
end

execute(search::DepthLimitedSearch, problem) =
    recursive_DLS(make_node(problem.initial_state), problem, search.limit)

function recursive_DLS(node, problem, limit)
    goal_test(problem, node.state) && return solution(node)
    limit == 0 && return :cutoff
    cutoff_occured = false
    for action in actions(problem, node.state)
        child = child_node(problem, node, action)
        result = recursive_DLS(child, problem, limit-1)
        result != :failure && result != :cutoff && return result
        cutoff_occured = (result == :cutoff)
    end
    return cutoff_occured ? :cutoff : :failure
end

"""
*IterativeDeepeningSearch* An uninformed graph search technique to reach goal.

Iteratively *DepthLimitedSearch* is used by incrementing the depth limit till a point comes
when there are no `:cutoff` return value.

pg. 89 Fig. 3.18 AIMA 3ed
"""
struct IterativeDeepeningSearch <: SearchAlgorithm
end

function execute(search::IterativeDeepeningSearch, problem)
    depth = 0
    while true
        result = execute(DepthLimitedSearch(depth), problem)
        result != :cutoff && return result
        depth += 1
    end
end

mutable struct GraphSearch{SQ, SET, S<:State, NT} <: SearchAlgorithm
    frontier::SQ
    explored::SET
    GraphSearch{SQ, SET, S, NT}() where {SQ, SET, S<:State, NT} =
        has_trait_queue(SQ{S}, NodeT{S, NT}) &&
        method_exists(insert_or_replace, (Function, SQ{S}, NodeT{S, NT})) &&
        new(SQ{S}(), has_trait_set(SET{S}()))
end

const GraphSearchBreadth{S <: State} = GraphSearch{AIMASearchFIFO, AIMASearchSet, S, :none}
const GraphSearchDepth{S <: State}   = GraphSearch{AIMASearchLIFO, AIMASearchSet, S, :none}
const GraphSearchUniformCost{S <: State} =
    GraphSearch{AIMASearchPQ, AIMASearchSet, S, :none}

const GraphSearchBestFirst{S <: State} =
    GraphSearch{AIMASearchPQ_G, AIMASearchSet, S, :greedy}
const GraphSearchAStar{S <: State} =
        GraphSearch{AIMASearchPQ_A, AIMASearchSet, S, :astar}

GraphSearchBreadth{S<:State}(::S)      = GraphSearchBreadth{S}()
GraphSearchDepth{S<:State}(::S)        = GraphSearchDepth{S}()
GraphSearchUniformCost{S<:State}(::S)  = GraphSearchUniformCost{S}()
GraphSearchBestFirst{S<:State}(::S)    = GraphSearchBestFirst{S}()
GraphSearchAStar{S<:State}(::S)        = GraphSearchAStar{S}()

function execute{SQ, SET, S<:State, T}(search::GraphSearch{SQ, SET, S, T}, problem)
    empty(search.frontier)
    empty(search.explored)
    node = NodeT(problem.initial_state, T)
    insert(search.frontier, node)

    while(true)
        isempty(search.frontier) && return failure(node)
        node = pop(search.frontier)
        goal_test(problem, node.state) && return solution(node)
        append(search.explored, node.state)
        for action in actions(problem, node.state)
            child = child_node(problem, node, action)
            if !(child.state in search.explored)
                insert_or_replace(x-> child < x, search.frontier, child)
            end
        end
    end
end

const AIMASearchSequence{S<:State} = Vector{Node{S}}
sort(sq::AIMASearchSequence) = sort!(sq)

struct RecursiveBestFirstSearch{SQ, S<:State} <: SearchAlgorithm
    SQ_t::Type
    RecursiveBestFirstSearch{SQ, S}() where {SQ, S<:State} =
        has_trait_sequence(SQ{S}, Node{S}) && new(Type(SQ{S}))
end

RecursiveBestFirstSearch{S<:State}(::S) = RecursiveBestFirstSearch{AIMASearchSequence, S}()

function execute(search::RecursiveBestFirstSearch, problem)
    node = make_node(problem.initial_state)
    status, val = RBFS(search, problem, node, Infinite(node.f))
    return status == :failure ? :failure : val
end

function RBFS(search, problem, node, f_limit)
    goal_test(problem, node.state) && return :success, solution(node)
    successors = search.SQ_t()

    for action in actions(problem, node.state)
        child = child_node(problem, node, action)
        insert(successors, child)
    end
    isempty(successors) && return :failure, Infinite(node.f)

    for s in successors
        s.f = max(s.path_cost + heuristic(problem, s.state), node.f)
    end

    while(true)
        sort(successors)
        best = successors[1]
        best.f > f_limit && return :failure, best.f
        alternative = length(successors) > 1 ? successors[2].f : Infinite(node.f)
        result = RBFS(search, problem, best, min(f_limit, alternative))
        result[1] == :success && return result
        best.f = result[2]
    end
end

struct StateNode{S<:State}
    state::S
    value::Number
end

StateNode(state::S) where S <: State = StateNode(state, 0)

isless(n1::StateNode, n2::StateNode)  = isless(n1.value, n2.value)
==(n1::StateNode, n2::StateNode) = n1.value == n2.value


successor_node(problem, state::State) = StateNode(state, state_value(problem, state))

const AIMASearchStateSequence{S<:State} = Vector{StateNode{S}}
sort(sq::AIMASearchStateSequence) = sort!(sq, rev=true)

struct HillClimbingSearch{SQ, S<:State} <: SearchAlgorithm
    SQ_t::Type
    plateau::Int
    HillClimbingSearch{SQ, S}(plateau::Int=typemax(Int)) where {SQ, S<:State} =
        has_trait_sequence(SQ{S}, StateNode{S}) && new(Type(SQ{S}), plateau)
end

HillClimbingSearch{S<:State}(::S, plateau::Int=typemax(Int)) =
    HillClimbingSearch{AIMASearchStateSequence, S}(plateau)

function execute(search::HillClimbingSearch, problem)
    node = StateNode(problem.initial_state, state_value(problem, problem.initial_state))
    cnt = 0
    while true
        successors = search.SQ_t()
        for state in successor_states(problem, node.state)
            successor = successor_node(problem, state)
            insert(successors, successor)
        end
        sort(successors)
        neighbor = successors[1]
        neighbor < node && return node.state
        if neighbor == node
            cnt >= search.plateau && return node.state
            cnt += 1
        elseif neighbor > node
            cnt = 0
        end
        node = neighbor
    end
end

struct SimulatedAnnealingSearch{SQ, S<:State} <: SearchAlgorithm
    SQ_t::Type
    schedule::Function
    SimulatedAnnealingSearch{SQ, S}(schedule::Function) where {SQ, S<:State} =
        has_trait_sequence(SQ{S}, Node{S}) && new(Type(SQ{S}), schedule)
end

SimulatedAnnealingSearch{S<:State}(::S,
    schedule::Function=(x-> x > 10000.0 ? 0.0 : 1.0/x)) =
    SimulatedAnnealingSearch{AIMASearchStateSequence, S}(schedule)

function execute(search::SimulatedAnnealingSearch, problem)
    node = StateNode(problem.initial_state, state_value(problem, problem.initial_state))
    schedule = search.schedule
    t = 1.0
    while true
        T = schedule(t)
        T <= 0.0000001 && return node.state
        successors = search.SQ_t()
        for state in successor_states(problem, node.state)
            successor = successor_node(problem, state)
            insert(successors, successor)
        end
        next = rand(successors)
        de = next.value - node.value
        p = de > 0 ? 1.0 : exp(de/T)
        if rand(Float64) < p
            node = next
        end
        t += 1.0
    end
end

#=

const Population{S<:State} = AIMASearchStateSequence{S}

weights(p::Population) = [s.value for s in p]

sum_weights(p::Population) = sum(weights(p))

function random_selection(p::Population)
    wt = rand()*sum_weights(p)
    sum = zero(p[1].value)
    for x in p
        sum += x.value
        sum >= wt && return x
    end
end

mutate{S <: State}(s::S)

struct GeneticAlgorithmSearch{SQ, S<:State} <: SearchAlgorithm
    fitnessFN::Function

    GeneticAlgorithmSearch{SQ, S}(fitnessFN::Function,
        p_mutate::Float64) where {SQ, S<:State} =
        has_trait_sequence(SQ{S}, Node{S}) && new(Type(SQ{S}), fitnessFN)
end

function execute(search::GeneticAlgorithmSearch, problem)
    inputs: population , a set of individuals
    FITNESS -F N, a function that measures the fitness of an individual
    while true
        new_population = search.SQ_t()
        for i = 1:length(population)
            x = random_selection(population)
            y = random_selection(population)
            child ← REPRODUCE (x , y)
        if (small random probability) then child ← M UTATE(child )
        add child to new population
        population ← new population
        until some individual is fit enough, or enough time has elapsed
    end
    return the best individual in population , according to FITNESS -F N
end

function reproduce(x , y)
    n = length(x)
    c = rand(1:n)
    return A PPEND (S UBSTRING(x , 1, c), S UBSTRING(y, c + 1, n))
end

=#
