using LinearAlgebra
using Plots

# Include module `AttitudeDynamics`
include("AttitudeDynamics.jl")
using .AttitudeDynamics

mutable struct CoordinateVectors
    x::Matrix
    y::Matrix
    z::Matrix
end

# Inertia matrix
Inertia = diagm([1.0, 1.0, 2.0])

# Disturbance torque
Torque = [0.0, 0.0, 0.0]


# Dynamics model (mutable struct)
dynamicsModel = AttitudeDynamics.DynamicsModel(Inertia, Torque)


# サンプリング時間
Ts = 1e-2

simulationTime = 15

# 時刻
time = 0:Ts:simulationTime

# Numbers of simulation data
simDataNum = round(Int, simulationTime/Ts) + 1;

# Coordinate system of a
coordinateA = AttitudeDynamics.CoordinateVector(
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

    omegaBA[:, loopCounter+1] = AttitudeDynamics.updateAngularVelocity(dynamicsModel, time[loopCounter], omegaBA[:, loopCounter], Ts, currentCoordB)

    quaternion[:, loopCounter+1] = AttitudeDynamics.updateQuaternion(omegaBA[:,loopCounter], quaternion[:, loopCounter], Ts)

    C = AttitudeDynamics.getTransformationMatrix(quaternion[:, loopCounter])

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