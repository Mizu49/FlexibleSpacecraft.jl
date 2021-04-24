using LinearAlgebra

# Define Dynamic Model
struct DynamicsModel
    # 慣性ダイアディック
    InertiaMatrix::Matrix

    # 外乱トルク
    DisturbanceTorque::Vector

    # 座標系Bのマトリクス
    corrdinateB::Matrix
end

# Equation of dynamics
function diffDynamics(model::DynamicsModel, currentTime, currentOmega)

    # skew matrix of angular velocity vector
    skewOmega = [
        0 -currentOmega[3] currentOmega[2]
        currentOmega[3] 0 -currentOmega[1]
        -currentOmega[2] currentOmega[1] 0]

    differential = inv(model.InertiaMatrix) * (model.DisturbanceTorque - model.corrdinateB' * model.InertiaMatrix * skewOmega * model.corrdinateB * model.corrdinateB' * currentOmega)

    return differential
end

# 4th order runge kutta method
function rungeKutta(f, currentTime, currentState, samplingTime)

    k1 = f(currentTime                 , currentState                      )
    k2 = f(currentTime + samplingTime/2, currentState + samplingTime/2 * k1)
    k3 = f(currentTime + samplingTime/2, currentState + samplingTime/2 * k2)
    k4 = f(currentTime + samplingTime  , currentState + samplingTime   * k3)

    nextState = currentState + samplingTime/6 * (k1 + 2*k2 + 2*k3 + k4)

    return nextState
end

# Inertia matrix
I = diagm(0 => [1.0, 1.0, 2.0])

# Disturbance torque
M = [0.0, 0.0, 0.0]

# Coordinate system of b
b = ones(Float64, 3, 3)

dynamicsModel = DynamicsModel(I, M, b)


# サンプリング時間
Ts = 1e-4
simulationTime = 10


simDataNum = round(Int, simulationTime/Ts)


omegaBA = zeros(3, simDataNum)
omegaBA[:,1] = [0.1 0.0 0.0]';


for loopCounter = 1:simDataNum-1

    # println(loopCounter)     
    diff = diffDynamics(dynamicsModel, Ts*loopCounter, omegaBA[:, loopCounter])

    omegaBA[:, loopCounter+1] = omegaBA[:, loopCounter] + diff * Ts

    println(omegaBA[:, loopCounter])
end