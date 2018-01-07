export  execute,
        Environment,
        Action, Action_NoOp,
        Percept,
        Agent,
        AgentProgram,
            TableDrivenAgentProgram,
            ReflexVacuumAgentProgram,
            SimpleReflexAgentProgram,
            ModelBasedReflexAgentProgram,
        State,
        Rule

using Compat

"""
An agent perceives an *environment* through sensors and acts with actuators.

Sensors provide agent the *percepts*, based on which the agent delivers
*actions*

Pg. 35, AIMA 3ed
"""
abstract type Environment end

"""
*AgentProgram* is an internal representation of an agent function with an
concrete implementation. While *agent function* can be abstract *AgentProgram*
provides clear direction to the implementation.

Pg. 35, AIMA 3ed
"""
abstract type AgentProgram end

"""
*Action* is an agent's response to the environment through actuators.

Although the representation of a string may suffice for more most sample
programs, an abstract type is introduced to emphacize the need for
providing a concrete type based on the environment or agent at hand.

In most problems we try to solve, the *Action* may be driven by the choice
of the *Environment*
"""
abstract type Action end

"""
*NoOp* is a directive where the agent does not take any futher action.
"""
struct NoOpActionType <: Action
  val::Symbol
  NoOpActionType() = new("NoOp")
end

const Action_NoOp = NoOpActionType()

"""
*Percept* is an input to the *Agent* from environment through sensors.

Although the representation of a Tuple may suffice for more most sample
programs, an abstract type is introduced to emphacize the need for
providing a concrete type based on the environment or agent at hand.

In most problems we try to solve, the *Percept* may be driven by the choice
of the *Environment*
"""
abstract type Percept end

"""
Given a *Percept* returns an *Action* apt for the agent.

Depending on the agent program the function may respond with different *Action*
evaluation strategies.
"""
execute{AP <: AgentProgram}(ap::AP, p::Percept) = error(E_ABSTRACT)

"""
*TableDrivenAgentProgram* is a simple model of an agent program where all
percept sequences are well-known ahead in time and can be organized as a
mapping from percepts to action.

Look at the corresponding execute method for *Action* evaluation strategy.

The implementation must have the following methods:

1. append - percept to the list of percepts seen my the AgentProgram
2. lookup - the percepts in the tables of the AgentProgram

Fig 2.7 Pg. 47, AIMA 3ed
"""
abstract type TableDrivenAgentProgram <: AgentProgram end

function execute(ap::TableDrivenAgentProgram, percept::Percept)
    append(ap.percepts, percept)
    action = lookup(ap.table, ap.percepts)
    return action
end

"""
*Rule* is an abstract representation of a framework that associates a *State*
condition to the appropriate action.

Definition of a condition can be implementation dependent.
"""
abstract type Rule end

"""
*State* is an internal evaluated position of the Environment. In the context
of the problem the *Environment* can be one of the stated states. Any input or'
action may lead to change in *Environment* state.
"""
abstract type State end

"""
*SimpleReflexAgentProgram* is a simple *Percept* to *Action* matching state
based rules.

It does not depend on the historical percept data.

It needs to implement two methods

1. interpret_input
2. rule_match

for all the concrete implementations.

3. rules - Will provide all the rules associated with the
AgentProgram.

Fig 2.10 Pg. 49, AIMA 3ed
"""
abstract type SimpleReflexAgentProgram <: AgentProgram end

function execute(ap::SimpleReflexAgentProgram, percept::Percept)
    state = interpret_input(percept);
    rule = rule_match(state, ap.rules);
    action = rule.action;
    return action;
end

interpret_input(percept) = error(E_ABSTRACT)
rule_match(state, rules) = error(E_ABSTRACT)

"""
*ModelBasedReflexAgentProgram* uses a model which is close to the
understanding of the world.

The *AgentProgram* updates the states based on the *Percepts* received.
"""
abstract type ModelBasedReflexAgentProgram <: AgentProgram end

function execute(ap::ModelBasedReflexAgentProgram, percept::Percept)
    ap.state = update_state(ap.state, ap.action, percept, ap.model)
    rule = rule_match(ap.state, ap.rules)
    ap.action = rule.action
    return ap.action
end

update_state(state, action, percept, model) = error(E_ABSTRACT)

"""
*SimpleProblemSolvingAgentProgram* formulates the goal then the problem.
Searches for the sequence of *Actions* that would solve the problem.

pg.67 AIMA 3e
"""
abstract type SimpleProblemSolvingAgentProgram <: AgentProgram end

function execute(ap::SimpleProblemSolvingAgentProgram, percept::Percept)
    #persistent: seq, an action sequence, initially empty
    #state, some description of the current world state
    #goal , a goal, initially null
    #problem, a problem formulation
    ap.state = update_state(ap.state, percept)
    if isempty(ap.seq)
        ap.goal = formulate_goal(ap.state)
        ap.problem = formulate_problem(ap.state, ap.goal)
        ap.seq = search(ap.problem)
        if ap.seq == failure
            return nothing
        end
    end
    action = first(ap.seq)
    ap.seq = rest(ap.seq)
    return action
end

update_state(state, percept) = error(E_ABSTRACT)
formulate_goal(state) = error(E_ABSTRACT)
formulate_problem(state, goal) = error(E_ABSTRACT)

"""
Agent perceives *Environment* through sensors and acts based on actuators.

While this code may not be there in the book this is a general outline one can
gather from the description in the book.

pg. 35 Fig.2.1 AIMA 3e
"""
struct Agent{AP}
    program::AP
    Agent{AP}(ap::AP) where{AP <: AgentProgram} = new(ap)
end

function execute(a::Agent, percept::Percept)
    action = execute(a.program, percept)
end
