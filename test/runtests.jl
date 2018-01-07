using Base.Test

try
    if Pkg.installed("AIMASamples") == nothing
        Pkg.add("AIMASamples")
    end
catch
    Pkg.clone("https://github.com/sambitdash/AIMASamples.jl.git")
end

Pkg.test("AIMASamples")
