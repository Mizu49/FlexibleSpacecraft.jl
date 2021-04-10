using LinearAlgebra

mutable struct SystemModel
    dimState::Int
    dimInput::Int
    dimOutput::Int

    # state space model
    A::Matrix
    B::Matrix
    C::Matrix
    D::Matrix
    E::Matrix
    F::Matrix

    # Constructor
    SystemModel() = new()
end

function RK4(system::SystemModel, Ts, state, input1, input2)
    
    k1 = Ts .* (system.A * state          + system.B * input1 + system.E * input2 + system.F)
    k2 = Ts .* (system.A * (state + k1/2) + system.B * input1 + system.E * input2 + system.F)
    k3 = Ts .* (system.A * (state + k2/2) + system.B * input1 + system.E * input2 + system.F)
    k4 = Ts .* (system.A * (state + k3/2) + system.B * input1 + system.E * input2 + system.F)
    
    return state + 1/6*(k1 + 2*k2 + 2*k3 + k4)            
end

# 棒の長さl
l = 3
# 棒の質量
m = 30

# サンプリング時間
Ts = 1e-3
simulationTime = 10

simDataNum = round(Int, simulationTime/Ts)

# 慣性行列I 時不変
I = diagm(0 => [(1/12)*m*l^2, (1/12)*m*l^2, 0])

# 外力トルク　一定値
M = [1 0 0]'


omegaBA = zeros(3, simDataNum)
omegaBA[:,1] = [1 0 0];

system = SystemModel()
system.A = zeros(3, 3)
system.B = zeros(3, 1)
system.E = zeros(3, 1)
system.F = M


for counter = 1:simDataNum-1
    omegaBA[:,counter+1] = RK4(system, Ts, omegaBA[:,counter], 0, 0)

    println(omegaBA[1, counter])    
end