using LinearAlgebra
using Plots

# Define Dynamic Model
mutable struct DynamicsModel
    # 慣性ダイアディック
    InertiaMatrix::Matrix

    # 外乱トルク
    DisturbanceTorque::Vector

end

# Struct of each coordinate
struct CoordinateVector
    x::Vector
    y::Vector
    z::Vector
end

mutable struct CoordinateVectors
    x::Matrix
    y::Matrix
    z::Matrix
end


# Equation of dynamics
function diffDynamics(model::DynamicsModel, currentTime, currentOmega, currentCoordB)

    # skew matrix of angular velocity vector
    skewOmega = [
        0 -currentOmega[3] currentOmega[2]
        currentOmega[3] 0 -currentOmega[1]
        -currentOmega[2] currentOmega[1] 0]

    differential = inv(model.InertiaMatrix) * (model.DisturbanceTorque - currentCoordB' * model.InertiaMatrix * skewOmega * currentCoordB * currentCoordB' * currentOmega)

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

function updateAngularVelocity(model::DynamicsModel, currentTime, currentOmega, samplingTime, currentCoordB)
    # Update the angular velocity vector using 4th order runge kutta method



    k1 = diffDynamics(model, currentTime                 , currentOmega                      , currentCoordB)
    k2 = diffDynamics(model, currentTime + samplingTime/2, currentOmega + samplingTime/2 * k1, currentCoordB)
    k3 = diffDynamics(model, currentTime + samplingTime/2, currentOmega + samplingTime/2 * k2, currentCoordB)
    k4 = diffDynamics(model, currentTime + samplingTime  , currentOmega + samplingTime   * k3, currentCoordB)

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


# Dynamics model (mutable struct)
dynamicsModel = DynamicsModel(Inertia, Torque)


# サンプリング時間
Ts = 1e-2

simulationTime = 15

# 時刻
time = 0:Ts:simulationTime

# Numbers of simulation data
simDataNum = round(Int, simulationTime/Ts) + 1;

# Coordinate system of a
coordinateA = CoordinateVector(
    [1, 0, 0],
    [0, 1, 0],
    [0, 0, 1]
)

# Coordinate system of b
coordinateB = CoordinateVectors(
    zeros(3, simDataNum),
    zeros(3, simDataNum),
    zeros(3, simDataNum),
)

coordinateB.x[:, 1] = coordinateA.x
coordinateB.y[:, 1] = coordinateA.y
coordinateB.z[:, 1] = coordinateA.z


omegaBA = zeros(3, simDataNum)
omegaBA[:,1] = [0.0 0.0 1.0]';

quaternion = zeros(4, simDataNum)
quaternion[:, 1] = [0.0 0.0 0.0 1.0]';

println("Begin simulation!")
for loopCounter = 1:simDataNum-1

    # println(loopCounter)    
    
    currentCoordB = hcat(coordinateB.x[:,loopCounter] , coordinateB.y[:,loopCounter], coordinateB.z[:,loopCounter])

    omegaBA[:, loopCounter+1] = updateAngularVelocity(dynamicsModel, time[loopCounter], omegaBA[:, loopCounter], Ts, currentCoordB)

    quaternion[:, loopCounter+1] = updateQuaternion(omegaBA[:,loopCounter], quaternion[:, loopCounter], Ts)

    C = getTransformationMatrix(quaternion[:, loopCounter])

    coordinateB.x[:, loopCounter+1] = C * coordinateA.x
    coordinateB.y[:, loopCounter+1] = C * coordinateA.y
    coordinateB.z[:, loopCounter+1] = C * coordinateA.z
    
    # println(omegaBA[:, loopCounter])
    # println(dynamicsModel.coordinateB[:,1])
end
println("Simulation is completed!")


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

plotIndex = 1000

coordFig = quiver(
    zeros(3), zeros(3), zeros(3),
    quiver = ( 
        [coordinateA.x[1], coordinateA.y[1], coordinateA.z[1]], 
        [coordinateA.x[2], coordinateA.y[2], coordinateA.z[2]], 
        [coordinateA.x[3], coordinateA.y[3], coordinateA.z[3]]),
    color = :black,
    linewidth = 4,
    xlims = (-1.2, 1.2),
    ylims = (-1.2, 1.2),
    zlims = (-1.2, 1.2),
    framestyle = :origin)

coordFig = quiver!(
    zeros(3), zeros(3), zeros(3),
    quiver = ( 
        [coordinateB.x[1,plotIndex], coordinateB.y[1,plotIndex], coordinateB.z[1,plotIndex]], 
        [coordinateB.x[2,plotIndex], coordinateB.y[2,plotIndex], coordinateB.z[2,plotIndex]], 
        [coordinateB.x[3,plotIndex], coordinateB.y[3,plotIndex], coordinateB.z[3,plotIndex]]),
    color = :blue,
    linewidth = 4,
    xlims = (-1.2, 1.2),
    ylims = (-1.2, 1.2),
    zlims = (-1.2, 1.2),
    framestyle = :origin)

display(coordFig)