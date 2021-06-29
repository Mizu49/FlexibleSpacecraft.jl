include("src/FlexibleSpacecraft.jl")
using .FlexibleSpacecraft

# Inertia matrix
Inertia = diagm([1.0, 1.0, 2.0])

# Disturbance torque
Torque = [0.02, 0.0, 0.0]


# Dynamics model (mutable struct)
dynamicsModel = AttitudeDynamics.DynamicsModel(Inertia, Torque)

# サンプリング時間
Ts = 1e-2

simulationTime = 60

# 時刻
time = 0:Ts:simulationTime

# Numbers of simulation data
simDataNum = round(Int, simulationTime/Ts) + 1;

# Coordinate system of a
coordinateA = TimeLine.CoordinateVector(
    [1, 0, 0],
    [0, 1, 0],
    [0, 0, 1]
)

# Coordinate system of b
coordinateB = TimeLine.initBodyCoordinate(simDataNum, coordinateA)

omegaBA = TimeLine.initAngularVelocity(simDataNum, [0, 0, 1])

quaternion = TimeLine.initQuaternion(simDataNum, [0, 0, 0, 1])

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

# fig1 = PlotGenerator.plotAngularVelocity(time, omegaBA)
# display(fig1)


fig2 = PlotGenerator.getCoordinateGif(time, Ts, coordinateA, coordinateB)
display(fig2)

# bodyCoordinate = TimeLine.extractCoordinateVector(10, Ts, coordinateB)
# fig3 = PlotGenerator.plotCoordinate(10, coordinateA, bodyCoordinate)
# display(fig3)
