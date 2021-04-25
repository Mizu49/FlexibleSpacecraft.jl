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


function updateAngularVelocity(model::DynamicsModel, currentTime, currentOmega, samplingTime)
    # Update the angular velocity vector using 4th order runge kutta method

    k1 = diffDynamics(model, currentTime                 , currentOmega                      )
    k2 = diffDynamics(model, currentTime + samplingTime/2, currentOmega + samplingTime/2 * k1)
    k3 = diffDynamics(model, currentTime + samplingTime/2, currentOmega + samplingTime/2 * k2)
    k4 = diffDynamics(model, currentTime + samplingTime  , currentOmega + samplingTime   * k3)

    nextOmega = currentOmega + samplingTime/6 * (k1 + 2*k2 + 2*k3 + k4)

    return nextOmega
end

# Inertia matrix
I = diagm(0 => [1.0, 1.0, 2.0])

# Disturbance torque
M = [0.0, 0.0, 0.0]

# Coordinate system of b
b = ones(Float64, 3, 3)

dynamicsModel = DynamicsModel(I, M, b)


# サンプリング時間
Ts = 1e-3
simulationTime = 60


simDataNum = round(Int, simulationTime/Ts)


omegaBA = zeros(3, simDataNum)
omegaBA[:,1] = [0.1 0.0 0.0]';


for loopCounter = 1:simDataNum-1

    # println(loopCounter)     

    currentTime = loopCounter*Ts

    omegaBA[:, loopCounter+1] = updateAngularVelocity(dynamicsModel, currentTime, omegaBA[:, loopCounter], Ts)

    println(omegaBA[:, loopCounter])
end