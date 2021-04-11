using LinearAlgebra

# サンプリング時間
Ts = 1e-3
simulationTime = 10

simDataNum = round(Int, simulationTime/Ts)

# 慣性ダイアディック
I = diagm(0 => [1.0, 1.0, 2.0])

# 外力トルク　一定値
M = [0 0 0]'




omegaBA = zeros(3, simDataNum)
omegaBA[:,1] = [1.0 0.0 1.0]';

cross(omegaBA[:,1], I)

for loopCounter = 1:simDataNum-1
    k1 = inv(I)*(M - dot(kron(omegaBA[:,loopCounter], I), omegaBA[:,loopCounter]) )


    println(omegaBA[1, counter])    
end