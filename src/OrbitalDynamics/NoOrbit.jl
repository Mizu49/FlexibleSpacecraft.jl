module NoOrbit

export NoOrbitModel

struct NoOrbitModel
    # no member variables
end

function get_angular_velocity(dynamicsmodel::NoOrbitModel)
    return 0
end

function get_velocity(dynamicsmodel::NoOrbitModel)
    return 0
end

function get_timeperiod(dynamicsmodel::NoOrbitModel; unit = "second")
    return 0
end

end
