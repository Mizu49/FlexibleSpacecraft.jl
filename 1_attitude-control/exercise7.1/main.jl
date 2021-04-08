using LinearAlgebra

function RK4(sysA, sysB, sysE, sysF, Ts, state, input1, input2)
    
    k1 = Ts * (sysA * state          + sysB * input1 + sysE * input2 + sysF)
    k2 = Ts * (sysA * (state + k1/2) + sysB * input1 + sysE * input2 + sysF)
    k3 = Ts * (sysA * (state + k2/2) + sysB * input1 + sysE * input2 + sysF)
    k4 = Ts * (sysA * (state + k3/2) + sysB * input1 + sysE * input2 + sysF)
    
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

for counter = 1:simDataNum-1
    omegaBA[:,counter+1] = RK4(0.0, 0.0, 0.0, M, Ts, omegaBA[:,counter], 0, 0)

    println(omegaBA[1, counter])    
end