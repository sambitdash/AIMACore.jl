export  rest, append,       # Sequence
        lookup,             # Table
        pop, insert,  # Queue
        Infinite

import  Base: isempty, first

const AIMASequence{T} = Vector{T}

Infinite(x) = typemax(x)

method_exists_throw(method, targs) =
    method_exists(method, targs) || throw(MethodError(method, targs))

function method_exists_throw(seq)
    foreach(seq) do x
        method_exists_throw(x...)
    end
    return true
end

has_trait_sequence(s) = has_trait_sequence(typeof(s), eltype(s)) && s
has_trait_sequence(ts, ds) = method_exists_throw([
        (empty, (ts,)),
        (isempty, (ts,)),
        (first, (ts,)),
        (rest, (ts,)),
        (append, (ts, ds)),
        (sort, (ts,)),
        (endof,(ts,)),
        (getindex, (ts, Integer))
    ])

has_trait_queue(s) = has_trait_queue(typeof(s), eltype(s)) && s
has_trait_queue(ts, ds) = method_exists_throw([
        (empty, (ts,)),
        (isempty, (ts,)),
        (pop, (ts,)),
        (insert, (ts, ds))
    ])

has_trait_set(s) = has_trait_set(typeof(s), eltype(s)) && s
has_trait_set(ts, ds) = method_exists_throw([
        (empty, (ts,)),
        (append, (ts, ds))
    ])

has_trait_table(t) = has_trait_table(typeof(t), eltype(keys(t)), eltype(values(t))) && t
has_trait_table(ts, ks, vs) = method_exists_throw(append, (ts, ks))

# Vector: As an AIMA Sequence and Queue
rest(seq::Vector) = (shift!(seq); seq)
append(seq::Vector, data) = (push!(seq, data))
empty(queue::Vector) = empty!(queue)
pop(queue::Vector) = shift!(queue)
insert(queue::Vector{T}, data::T) where{T} = append(queue, data)
sort(seq::Vector) = sort!(sq)

#Set: As an AIMA Set
append(s::Set, data) = push!(s, data)
empty(s::Set) = empty!(s)

#Dict: as an AIMA Table
lookup(table::Dict, key) = table[key]
