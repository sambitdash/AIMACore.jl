using Base.Test

try
    if Pkg.installed("AIMASamples") == nothing
        Pkg.add("AIMASamples")
    end
catch
    Pkg.clone("https://github.com/sambitdash/AIMASamples.jl")
end

include(joinpath(Pkg.dir("AIMASamples"), "test", "runtests.jl"))
