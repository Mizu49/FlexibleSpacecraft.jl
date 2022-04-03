using Test

include("../../src/FlexibleSpacecraft.jl")
using .FlexibleSpacecraft

paramfile = "../solararray.yml"

𝐌 = [
    10 0
    0 20
]
𝐊 = [
    10 0
    0 10000
]
𝐂 = zeros(2, 2)

𝐄ctrl = []
𝐄dist = []
𝐄coupling = [
    1 0 0
    0 1 0
]

