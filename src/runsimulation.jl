"""
    function runsimulation
"""
function runsimulation(model, ECI_frame::Frame, initvalue::TimeLine.InitData, distconfig::DisturbanceConfig, simulation_time::Real, Tsampling::Real)::TimeLine.DataTimeLine

    # Numbers of simulation data
    datanum = floor(Int, simulation_time/Tsampling) + 1;

    # Initialize data array
    simdata = TimeLine.DataTimeLine(initvalue, Tsampling, datanum)

    for loopCounter = 0:datanum - 1

        # Update current time (second)
        currenttime = simdata.time[loopCounter + 1]

        # Update current attitude
        C = ECI2BodyFrame(simdata.quaternion[:, loopCounter + 1])
        currentbodyframe = C * ECI_frame

        simdata.bodyframes[loopCounter + 1] = currentbodyframe

        # Disturbance torque
        disturbance = disturbanceinput(distconfig)

        # Time evolution of the system
        if loopCounter != datanum - 1

            # Update angular velocity
            simdata.angularvelocity[:, loopCounter + 2] = calc_angular_velocity(model, simdata.time[loopCounter + 1], simdata.angularvelocity[:, loopCounter + 1], Tsampling, currentbodyframe, disturbance)

            # Update quaternion
            simdata.quaternion[:, loopCounter + 2] = calc_quaternion(simdata.angularvelocity[:,loopCounter + 1], simdata.quaternion[:, loopCounter + 1], Tsampling)

        end

    end

    return simdata
end
