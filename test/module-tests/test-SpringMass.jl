using Test

include("../../src/FlexibleSpacecraft.jl")
using .FlexibleSpacecraft

paramfile = "../solararray.yml"

M = [
    100 0
    0 50
]
K = [
    6e4 -1e4
    -1e4 1e4
]
C_Ma = 0
C_Mb = 0
C_Ka = 250
C_Kb = 50
C = [
    C_Ma+C_Kb+C_Kb -C_Kb
    -C_Kb C_Mb+C_Kb
]

Ectrl = []
Edist = []
Ecoupling = [
    1 0 0
    0 1 0
]

# Create representation of the system in physical coordinate
physicalsystem = SpringMass.PhysicalSystem(M, C, K)

modalsystem = SpringMass._mode_decomposition(physicalsystem)
println(modalsystem)
