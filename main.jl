include("src/FlexibleSpacecraft.jl")
using .FlexibleSpacecraft

# inertia matrix
inertia = diagm([1, 1, 1])

# Orbit Radius [m]
orbitalRadius = 4e+5

# Assume as a circular orbit
gravitationalConstant = 3.9879e+14
orbitAngularVelocity  = sqrt(gravitationalConstant/orbitalRadius^3)


# Dynamics model (mutable struct)
model = RigidBodyAttitudeDynamics.DynamicsModel(inertia)

# Sampling period of simulation (second)
Ts = 1e-2

# Time length of simulation (second)
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

omegaBA = TimeLine.init_angular_velocity_array(simu_data_num, [0, 0, 0])

quaternion = TimeLine.init_quaternion_array(simu_data_num, [0, 0, 0, 1])

println("Begin simulation!")
for loopCounter = 0:simu_data_num - 2

    # println(loopCounter + 1)

    # Extract body fixed frame at current time step
    currentCoordB = hcat(coordinateB.x[:,loopCounter + 1] , coordinateB.y[:,loopCounter + 1], coordinateB.z[:,loopCounter + 1])

    # Disturbance torque
    disturbance = RigidBodyAttitudeDynamics.gravity_gradient_torque(inertia, orbitAngularVelocity, quaternion[:, loopCounter + 1])

    omegaBA[:, loopCounter + 2] = RigidBodyAttitudeDynamics.calc_angular_velocity(model, time[loopCounter + 1], omegaBA[:, loopCounter + 1], Ts, currentCoordB, disturbance)

    #quaternion[:, loopCounter + 2] = RigidBodyAttitudeDynamics.calc_quaternion(omegaBA[:,loopCounter + 1], quaternion[:, loopCounter + 1], Ts)
    quaternion[:, loopCounter + 2] = RigidBodyAttitudeDynamics.calc_quaternion_orbit(omegaBA[:,loopCounter + 1], quaternion[:, loopCounter + 1], Ts, orbitAngularVelocity)

    # Divergence judgement
    if isnan(quaternion[1, loopCounter + 2]) || isnan(quaternion[2, loopCounter + 2]) || isnan(quaternion[3, loopCounter + 2]) ||isnan(quaternion[4, loopCounter + 2])
        println("error. quaternion diverged at loopCounter = ",loopCounter)
        break
    end
    C = RigidBodyAttitudeDynamics.calc_transformation_matrix(quaternion[:, loopCounter + 1])

    coordinateB.x[:, loopCounter + 2] = C * coordinateA.x
    coordinateB.y[:, loopCounter + 2] = C * coordinateA.y
    coordinateB.z[:, loopCounter + 2] = C * coordinateA.z

end
println("Simulation is completed!")

fig1 = PlotGenerator.angular_velocity(time, omegaBA)
display(fig1)


fig2 = PlotGenerator.frame_gif(time, Ts, coordinateA, coordinateB)
display(fig2)

bodyCoordinate = TimeLine.get_coordinate(10, Ts, coordinateB)
fig3 = PlotGenerator.bodyframe(10, coordinateA, bodyCoordinate)
display(fig3)
