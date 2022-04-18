using Test

include("../../src/FlexibleSpacecraft.jl")
using .FlexibleSpacecraft
using Plots

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

Ectrl = zeros(2)
Edist = zeros(2)
Ecoupling = [
    1 0 0
    0 1 0
]

# Create representation of the system in physical coordinate
physicalsystem = PhysicalSystem(M, C, K)
# Convert representation of the system in modal coordinate
modalsystem = physical2modal(physicalsystem)

systemmodel = SpringMassModel(physicalsystem, Ecoupling, Ectrl, Edist)

model = StateSpace(systemmodel)

# Test the simulation feature
Ts = 1e-3
times = 0:Ts:50

datanum = size(times, 1)

response = [zeros(4) for _ in 1:datanum]
response[1] = [10e-3, 10e-3, 0, 0]

for cnt = 1:datanum-1
    response[cnt+1] = updatestate(model, Ts, times[cnt], response[cnt], zeros(3), 0, 0)
end

plot(times, getindex.(response, 1))
