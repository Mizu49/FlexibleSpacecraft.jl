using LinearAlgebra
using Plots

# Include module `AttitudeDynamics`
include("AttitudeDynamics.jl")
import .AttitudeDynamics

# Include module `TimeLine`
include("TimeLine.jl")
import .TimeLine as tl

# Include module `PlotGenerator`
include("PlotGenerator.jl")
import .PlotGenerator as plt


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
coordinateA = tl.CoordinateVector(
    [1, 0, 0],
    [0, 1, 0],
    [0, 0, 1]
)

# Coordinate system of b
coordinateB = tl.initBodyCoordinate(simDataNum, coordinateA)

omegaBA = tl.initAngularVelocity(simDataNum, [0, 0, 1])

quaternion = tl.initQuaternion(simDataNum, [0, 0, 0, 1])

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
    
end
println("Simulation is completed!")

plt.plotAngularVelocity(time, omegaBA)

plotTime = 10
bodyCoord = tl.extractCoordinateVector(plotTime, Ts, coordinateB)
plt.plotCoordinate(plotTime, coordinateA, bodyCoord)


dataNum = size(time, 1)
anim = @animate for index in 1:dataNum

    bodyCoordinate = tl.extractCoordinateVector(time[index], Ts, coordinateB)

    plt.plotCoordinate(time[index], coordinateA, bodyCoordinate)
end every 10
gif(anim, "attitude.gif", fps = 120)
