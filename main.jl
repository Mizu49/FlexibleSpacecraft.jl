include("src/FlexibleSpacecraft.jl")
using .FlexibleSpacecraft

# inertia matrix
inertia = diagm([1.0, 1.0, 2.0])

# Disturbance disturbance
disturbance = [0.02, 0.0, 0.0]


# Dynamics model (mutable struct)
model = RigidBodyAttitudeDynamics.model(inertia, disturbance)

# サンプリング時間
Ts = 1e-2

simulation_time = 60

# 時刻
time = 0:Ts:simulation_time

# Numbers of simulation data
simu_data_num = round(Int, simulation_time/Ts) + 1;

# Coordinate system of a
coordinateA = TimeLine.Coordinate(
    [1, 0, 0],
    [0, 1, 0],
    [0, 0, 1]
)

# Coordinate system of b
coordinateB = TimeLine.init_coordinate_array(simu_data_num, coordinateA)

omegaBA = TimeLine.init_angular_velocity_array(simu_data_num, [0, 0, 1])

quaternion = TimeLine.init_quaternion_array(simu_data_num, [0, 0, 0, 1])

println("Begin simulation!")
for loopCounter = 1:simu_data_num-1

    # println(loopCounter)

    currentCoordB = hcat(coordinateB.x[:,loopCounter] , coordinateB.y[:,loopCounter], coordinateB.z[:,loopCounter])

    omegaBA[:, loopCounter+1] = RigidBodyAttitudeDynamics.calc_angular_velocity(model, time[loopCounter], omegaBA[:, loopCounter], Ts, currentCoordB)

    quaternion[:, loopCounter+1] = RigidBodyAttitudeDynamics.calc_quaternion(omegaBA[:,loopCounter], quaternion[:, loopCounter], Ts)

    C = RigidBodyAttitudeDynamics.calc_transformation_matrix(quaternion[:, loopCounter])

    coordinateB.x[:, loopCounter+1] = C * coordinateA.x
    coordinateB.y[:, loopCounter+1] = C * coordinateA.y
    coordinateB.z[:, loopCounter+1] = C * coordinateA.z

end
println("Simulation is completed!")

fig1 = PlotGenerator.angular_velocity(time, omegaBA)
display(fig1)


fig2 = PlotGenerator.frame_gif(time, Ts, coordinateA, coordinateB)
display(fig2)

bodyCoordinate = TimeLine.extractCoordinateVector(10, Ts, coordinateB)
fig3 = PlotGenerator.bodyframe(10, coordinateA, bodyCoordinate)
display(fig3)
