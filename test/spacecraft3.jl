# simulation parameter setting script

spacecraft = Dict()

## simulation configuration
spacecraft["config"] = Dict(
    "name" => "example spacecraft",
    "note" => "This model is for test purpose",
    "sampling time" => 1e-3,
    "time length" => 5000
)

## initlal value of the attitude
spacecraft["initial value"] = Dict(
    "roll-pitch-yaw" => [0.0, 0.0, 0.0],
    "angular velocity" => [0.0, 0.0, 0.0]
)

## spacecraft attitdue dynamics model configuration
spacecraft["attitude dynamics"] = Dict(
    "model" => "Rigid body",
    "inertia" => [
        45000.0 0.0 0.0
        0.0 1200.0 0.0
        0.0 0.0 50000.0
    ]
)

## attitude disturbance configuration
spacecraft["disturbance"] = Dict(
    "constant torque" => [0.0, 0.0, 0.0],
    "gravitational torque" => true,
    "step trajectory" => nothing
)

## Orbital dynamics
spacecraft["Orbit"] = Dict(
    # Specify the model used in the simulation of orbital motion
    "Dynamics model" => "Circular",
    # Orbital elements
    "Orbital elements" => Dict(
        "right ascension" => 0.0,
        "inclination" => 0.0,
        "semimajor axis" => 6371e3,
        "eccentricity" => 0.0,
        "argument of perigee" => 0.0,
        "true anomaly at epoch" => 0.0
    ),
    # Note for this parameter setting
    "OrbitInfo" => "Test orbit parameters"
)

## Flexible appendages
spacecraft["appendage"] = Dict(
    "modeling" => "none",
)

## Attitude control
spacecraft["attitude controller"] = Dict(
    "strategy" => "none"
)
