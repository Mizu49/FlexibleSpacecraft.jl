using LinearAlgebra
using Plots

# Define Dynamic Model
mutable struct DynamicsModel
    # 慣性ダイアディック
    InertiaMatrix::Matrix

    # 外乱トルク
    DisturbanceTorque::Vector

    # 座標系Bのマトリクス
    coordinateB::Matrix
end

# Equation of dynamics
function diffDynamics(model::DynamicsModel, currentTime, currentOmega)

    # skew matrix of angular velocity vector
    skewOmega = [
        0 -currentOmega[3] currentOmega[2]
        currentOmega[3] 0 -currentOmega[1]
        -currentOmega[2] currentOmega[1] 0]

    differential = inv(model.InertiaMatrix) * (model.DisturbanceTorque - model.coordinateB' * model.InertiaMatrix * skewOmega * model.coordinateB * model.coordinateB' * currentOmega)

    return differential
end


# Equation of Quaternion
function diffQuaternion(omega, quaternion)

    OMEGA = [
        0 omega[3] -omega[2] omega[1]
        -omega[3] 0 omega[1] omega[2]
        omega[2] -omega[1] 0 omega[3]
        -omega[1] -omega[2] -omega[3] 0
    ]

    differential = 1/2 * OMEGA * quaternion

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


function updateQuaternion(currentOmega, currentQuaternion, samplingTime)
    # Update the quaterion vector using 4th order runge kutta method

    k1 = diffQuaternion(currentOmega, currentQuaternion                      );
    k2 = diffQuaternion(currentOmega, currentQuaternion + samplingTime/2 * k1);
    k3 = diffQuaternion(currentOmega, currentQuaternion + samplingTime/2 * k2);
    k4 = diffQuaternion(currentOmega, currentQuaternion + samplingTime   * k3);

    nextQuaternion = currentQuaternion + samplingTime/6 * (k1 + 2*k2 + 2*k3 + k4);

    return nextQuaternion    
end

function getTransformationMatrix(q)

    # Check if the quaterion satisfies its constraint
    try
        constraint = q[1]^2 + q[2]^2 + q[3]^2 + q[4]^2

    catch constraint

        if constraint < 0.995
            error("Quaternion does not satisfy constraint")
        elseif constraint > 1.005
            error("Quaternion does not satisfy constraint")
        end
    end

    C = [
        q[1]^2 - q[2]^2 - q[3]^2 + q[4]^2  2*(q[1]*q[2] + q[3]*q[4])          2*(q[1]*q[3] - q[2]*q[4])
        2*(q[2]*q[1] - q[3]*q[4])          q[2]^2 - q[3]^2 - q[1]^2 + q[4]^2  2*(q[2]*q[3] + q[1]*q[4])
        2*(q[3]*q[1] + q[2]*q[4])          2*(q[3]*q[2] - q[1]*q[4])          q[3]^2 - q[1]^2 - q[2]^2 + q[4]^2
    ]

    return C
end

# Inertia matrix
Inertia = diagm([1.0, 1.0, 2.0])

# Disturbance torque
Torque = [0.0, 0.0, 0.0]

# Coordinate system of a
coordinateA = diagm([1.0, 1.0, 1.0])

# Coordinate system of b
coordinateB = diagm([1.0, 1.0, 1.0])

# Dynamics model (mutable struct)
dynamicsModel = DynamicsModel(Inertia, Torque, coordinateB)


# サンプリング時間
Ts = 1e-2
simulationTime = 30

# 時刻
time = 0:Ts:simulationTime

# Numbers of simulation data
simDataNum = round(Int, simulationTime/Ts) + 1;

omegaBA = zeros(3, simDataNum)
omegaBA[:,1] = [0.0 0.0 1.0]';

quaternion = zeros(4, simDataNum)
quaternion[:, 1] = [0.0 0.0 0.0 1.0]';


for loopCounter = 1:simDataNum-1

    # println(loopCounter)     

    omegaBA[:, loopCounter+1] = updateAngularVelocity(dynamicsModel, time[loopCounter], omegaBA[:, loopCounter], Ts)

    quaternion[:, loopCounter+1] = updateQuaternion(omegaBA[:,loopCounter], quaternion[:, loopCounter], Ts)

    C = getTransformationMatrix(quaternion[:, loopCounter])

    dynamicsModel.coordinateB = C * coordinateA
    
    # println(omegaBA[:, loopCounter])
    # println(dynamicsModel.coordinateB[:,1])
end



fig1 = plot(time, omegaBA[1, :], 
    xlabel ="Time [s]",
    ylabel ="omega [rad/s]",
    label  ="Angular velocity 1",
)

fig2 = plot(time, omegaBA[2, :], 
    xlabel ="Time [s]",
    ylabel ="omega [rad/s]",
    label  ="Angular velocity 2",
)

fig3 = plot(time, omegaBA[3, :], 
    xlabel ="Time [s]",
    ylabel ="omega [rad/s]",
    label  ="Angular velocity 3",
)

# Graph of the angular velocity
hoge = plot(fig1, fig2, fig3, layout = (3, 1), legend = true)
display(hoge)

