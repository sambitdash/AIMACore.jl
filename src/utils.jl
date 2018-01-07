export  AIMASequence, AbstractSequence,
            rest, append,
        AIMATable, AbstractTable,
            lookup,
        AIMAQueue, AbstractQueue,
            empty, pop, insert

import  Base: isempty, first
#=
Methods needed in a sequence. Sequence shall not mean an Array or Vector.
The domain can decide the sequence based on the need. It shall have following
Methods
=#

abstract type AbstractSequence{T} end

const AIMASequence{T} = Union{AbstractSequence{T}, Vector{T}}

isempty(seq::AbstractSequence) = error(E_ABSTRACT)
first(seq::AbstractSequence) = error(E_ABSTRACT)
rest(seq::AbstractSequence) = error(E_ABSTRACT)
append(seq::AbstractSequence{T}, data::T) = error(E_ABSTRACT)

rest(seq::Vector) = (shift!(seq); seq)
append(seq::Vector, data) = (push!(seq, data))

#=
Methods needed in a queue. Queue shall not mean an Array or Vector.
The domain can decide the sequence based on the need. It shall have following
Methods
=#

#@compat abstract type Queue end

#@compat abstract type PriorityQueue end

abstract type AbstractQueue{T} end

const AIMAQueue{T} = Union{AbstractQueue{T}, Vector{T}}

isempty(queue::AbstractQueue) = error(E_ABSTRACT)
empty(queue::AbstractQueue) = error(E_ABSTRACT)
pop(queue::AbstractQueue) = error(E_ABSTRACT)
insert(queue::AbstractQueue{T}, data::T) = error(E_ABSTRACT)

empty(queue::Vector) = empty!(queue)
pop(queue::Vector) = shift!(queue)
insert(queue::Vector, data) = append(queue, data)

append(set::Set, data) = push!(set, data)


abstract type AbstractTable{K, V} end

const AIMATable{K, V} = Union{AbstractTable{K, V}, Dict{K, V}}

lookup(table::AbstractTable,key) = error(E_ABSTRACT)
lookup(table::Dict, key) = table[key]
