using Test
using Pkg

Pkg.clone("https://github.com/sambitdash/AIMASamples.jl")

import AIMASamples; joinpath(dirname(pathof(AIMASamples)), "test", "runtests.jl")
