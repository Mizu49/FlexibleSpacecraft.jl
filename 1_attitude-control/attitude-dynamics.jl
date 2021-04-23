using LinearAlgebra


function rungeKutta(f, currentTime, currentState, samplingTime)
    k1 = f(currentTime                 , currentState                      )
    k2 = f(currentTime + samplingTime/2, currentState + samplingTime/2 * k1)
    k3 = f(currentTime + samplingTime/2, currentState + samplingTime/2 * k2)
    k4 = f(currentTime + samplingTime  , currentState + samplingTime   * k3)

    nextState = currentState + samplingTime/6 * (k1 + 2*k2 + 2*k3 + k4)

    return nextState
end

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


for loopCounter = 1:simDataNum-1
    println(loopCounter)    
end