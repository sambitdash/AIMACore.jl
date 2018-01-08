export  Problem,
            result, step_cost, goal_test, actions,
        Node,
            solution, failure, isless,
        SearchAlgorithm,
            BreadthFirstSearch,
                execute,
            UniformCostSearch,
            DepthLimitedSearch,
        AIMASearchFIFO,
        AIMASearchPQ,
        AIMASearchSet

using Compat

using DataStructures

import Base: isless, in

@compat abstract type Problem end

result(problem::Problem, state::State, action::Action) = error(E_ABSTRACT)
step_cost(problem::Problem, state::State, action::Action) = error(E_ABSTRACT)
goal_test(problem::Problem, state::State) = error(E_ABSTRACT)
actions(problem::Problem,state::State) = error(E_ABSTRACT)

search(problem::Problem) = execute(problem.search_algorithm, problem)

function child_node(problem, parent, action)
    state = result(problem, parent.state, action)
    path_cost = parent.path_cost + step_cost(problem, parent.state, action)
    return Node(state, parent, action, path_cost)
end

mutable struct Node{S<:State}
    state::S
    parent::Nullable{Node{S}}
    action::Action
    path_cost::Number
end

Node{S<:State}(state::S, action::Action=Action_NoOp, path_cost::Number=0)=
    Node(state,Nullable{Node{S}}(),action,path_cost)
Node{S<:State}(state::S, parent::Node{S}, action::Action=Action_NoOp, path_cost::Number=0)=
    Node(state, Nullable(parent), action, path_cost)

make_node{S<:State}(state::S) = Node(state)

isless{S<:State}(n1::Node{S}, n2::Node{S}) = isless(n1.path_cost, n2.path_cost)

"""
The default implementation of *solution* is returning the sequence of nodes.
This may not be ideal for many implementations. They may provide a more
elaborate reporting as well.
"""
function solution{S<:State}(node::Node{S})
    nodes = Vector{Node{S}}()
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
failure(node::Node) = error(E_NODE_NOT_REACHABLE)

@compat abstract type SearchAlgorithm end

const AIMASearchSet{S<:State} = Set{S}

const AIMASearchFIFO{S<:State} = Vector{Node{S}}

function in{S<:State}(state::S, s::AIMASearchFIFO{S})
    for n in s
        state == n.state && return true
    end
    return false
end

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
        new(has_trait_sequence(FIFO{S}()), has_trait_set(SET{S}()))
end

BreadthFirstSearch{S<:State}(::S) = BreadthFirstSearch{AIMASearchFIFO, AIMASearchSet, S}()

function execute(search::BreadthFirstSearch, problem::Problem)
    node = Node(problem.initial_state)

    if goal_test(problem, node.state)
        return solution(node)
    end

    insert(search.frontier, node)

    while(true)
        if isempty(search.frontier)
            return failure(node)
        end

        node = pop(search.frontier)
        append(search.explored, node.state)

        for action in actions(problem, node.state)
            child = child_node(problem, node, action)
            if !(child.state in search.explored) && !(child.state in search.frontier)
                if goal_test(problem, child.state)
                    return solution(child)
                end
            end
            insert(search.frontier, child)
        end
    end
end

const AIMASearchPQ{S<:State} = PriorityQueue{S, Node{S}}

pop{S<:State}(pq::AIMASearchPQ{S}) = ((key,val) = peek(pq); dequeue!(pq); val)
insert{S<:State}(pq::AIMASearchPQ{S}, node::Node{S}) = enqueue!(pq, node.state, node)
replace{S<:State}(pq::AIMASearchPQ{S}, node::Node{S}) = (pq[node.state] = node)

in{S<:State}(state::S, pq::AIMASearchPQ{S}) = haskey(pq, state)

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
        method_exists(in, (S, PQ{S})) &&
        new(PQ{S}(), has_trait_set(SET{S}()))
end

UniformCostSearch{S<:State}(::S) = UniformCostSearch{AIMASearchPQ, AIMASearchSet, S}()

function execute(search::UniformCostSearch, problem) #returns a solution, or failure
    node = Node(problem.initial_state)

    if goal_test(problem, node.state)
        return solution(node)
    end
    insert(search.frontier, node)
    while(true)
        if isempty(search.frontier)
            return failure(node)
        end

        node = pop(search.frontier)

        if goal_test(problem, node.state)
            return solution(node)
        end

        append(search.explored, node.state)

        for action in actions(problem, node.state)
            child = child_node(problem, node, action)
            if !(child.state in search.explored) && !(child.state in search.frontier)
                insert(search.frontier, child)
            elseif (child.state in search.frontier) &&
                    (child.path_cost < get(search.frontier, child.state,
                                           Node(problem.initial_state)).path_cost)
                replace(search.frontier,child)
            end
        end
    end
end

struct DepthLimitedSearch <: SearchAlgorithm
    limit::Int
end

function execute(search::DepthLimitedSearch, problem)
    return recursive_DLS(make_node(problem.initial_state), problem,search.limit)
end

function recursive_DLS(node, problem, limit)
    if goal_test(problem, node.state)
        return solution(node)
    elseif limit==0
        return :cutoff
    else
        cutoff_occured = false
        for action in actions(problem, node.state)
            child = child_node(problem, node, action)
            result = recursive_DLS(child, problem, limit-1)
            if result == :cutoff
                cutoff_occured = true
            elseif result == :failure
                return result
            end
        end
        if cutoff_occured
            return :cutoff
        else
            return :failure
        end
    end
end

#=
type RecursiveBestFirstSearch <: SearchAlgorithm
  const
  limit::Int
end

function execute(search::RecursiveBestFirstSearch, problem)
  return RBFS(search, problem, make_node(problem.initial_state), INFINITE)
end

function RBFS(problem, node, f_limit)
  if goal_test(problem, node.state)
    return solution(node)
  end

  for action in actions(problem, node.state)
    child = child_node(problem, node, action)
    insert(successors, child)
  end

  if isempty(successors)
    return failure, INFINITE
  end
  for s in successors
    s.f = max (s.g +s.h, node.f)
  end
  while(true)
    best = pop(successors)
    if best.f > f_limit
      return failure, best.f
    end
    alternative = pop(successors).f
    result, best.f ← RBFS(problem, best, min( f_limit, alternative))
    if result != failure
      return result
    end
  end
end
=#
