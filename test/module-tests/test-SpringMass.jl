using Test

include("../../src/FlexibleSpacecraft.jl")
using .FlexibleSpacecraft

paramfile = "../solararray.yml"

𝐌 = [
    100 0
    0 50
]
𝐊 = [
    6e4 -1e4
    -1e4 1e4
]
C_Ma = 0
C_Mb = 0
C_Ka = 250
C_Kb = 50
𝐂 = [
    C_Ma+C_Kb+C_Kb -C_Kb
    -C_Kb C_Mb+C_Kb
]

𝐄ctrl = []
𝐄dist = []
𝐄coupling = [
    1 0 0
    0 1 0
]

modalmat = SpringMass._mode_decomposition(𝐌, 𝐂, 𝐊)
println(modalmat)
