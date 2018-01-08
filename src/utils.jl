export  rest, append,       # Sequence
        lookup,             # Table
        pop, insert  # Queue

import  Base: isempty, first

method_exists_throw(method, targs) =
    method_exists(method, targs) || throw(MethodError(method, targs))

function method_exists_throw(seq)
    foreach(seq) do x
        method_exists_throw(x...)
    end
    return true
end

has_trait_sequence(s) = has_trait_sequence(typeof(s), eltype(s)) && s

function has_trait_sequence(ts, ds)
    seq = [
        (isempty, (ts,)),
        (first, (ts,)),
        (rest, (ts,)),
        (append, (ts, ds))
    ]
    return method_exists_throw(seq)
end

has_trait_queue(s) = has_trait_queue(typeof(s), eltype(s)) && s

function has_trait_queue(ts, ds)
    seq = [
        (isempty, (ts,)),
        (pop, (ts,)),
        (insert, (ts, ds)),
        (replace, (ts, ds))
    ]
    method_exists_throw(seq)
    return true
end

has_trait_set(s) = has_trait_set(typeof(s), eltype(s)) && s
has_trait_set(ts, ds) = method_exists_throw(append, (ts, ds))

has_trait_table(t) = has_trait_table(typeof(t), eltype(keys(t)), eltype(values(t))) && t
has_trait_table(ts, ks, vs) = method_exists_throw(append, (ts, ks))

# Vector: As an AIMA Sequence and Queue
rest(seq::Vector) = (shift!(seq); seq)
append(seq::Vector, data) = (push!(seq, data))
#empty(queue::Vector) = empty!(queue)
pop(queue::Vector) = shift!(queue)
insert(queue::Vector{T}, data::T) where{T} = append(queue, data)

#Set: As an AIMA Set
append(set::Set, data) = push!(set, data)

#Dict: as an AIMA Table
lookup(table::Dict, key) = table[key]
