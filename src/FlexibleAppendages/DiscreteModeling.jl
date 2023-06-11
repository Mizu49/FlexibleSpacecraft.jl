"""
    DiscreteModeling

submodule that contains all features for spring-mass modeling of flexible appendages of spacecraft
"""
module DiscreteModeling

using LinearAlgebra, StaticArrays
using ..UtilitiesBase, ..AppendagesBase

export physical2modal, PhysicalSystem, ModalSystem, SpringMassModel, StateSpace, updatestate, modalstate2physicalstate, physicalstate2modalstate, SpringMassParams

include("DiscreteSpringMass.jl")
include("StateSpaceModel.jl")


"""
    defmodel(params::SpringMassParams)

function that incorporates the model formulation process

# Argument

* `params::SpringMassParams`: struct that incorporates the parameter setting of the spring-mass structural system
"""
function defmodel(params::SpringMassParams)

    # Create representation of the system in physical coordinate
    physicalsystem = PhysicalSystem(params.M, params.D, params.K)
    # Convert representation of the system in modal coordinate
    modalsystem = physical2modal(physicalsystem)

    systemmodel = SpringMassModel(modalsystem, params.Ecoupling, params.Econtrol, params.Edisturbance)

    model = StateSpace(systemmodel)

    return model
end

"""
    defmodel(paramdict::AbstractDict)

# Argument

* `paramdict::AbstractDict`: dictionary that incorporates the parameter setting of the spring-mass structural system. This dictionary works with the parameter setting YAML file
"""
function defmodel(paramdict::AbstractDict)

    DOF = paramdict["system"]["DOF"]

    # read the parameters from the dictionary
    M = load_matrix(paramdict["system"]["mass"])
    K = load_matrix(paramdict["system"]["stiffness"])

    if paramdict["system"]["damping"]["config"] == "Rayleigh"
        alpha = paramdict["system"]["damping"]["alpha"]
        beta = paramdict["system"]["damping"]["beta"]
        D = alpha * M + beta * K
    else
        error("damping configuration for \"$(paramdict["system"]["damping"]["config"])\" not found")
    end

    dimcontrolinput = paramdict["system"]["control input"]["dimension"]
    Ectrl = load_matrix(paramdict["system"]["control input"]["coefficient"])

    dimdistinput = paramdict["system"]["control input"]["dimension"]
    Edist = load_matrix(paramdict["system"]["disturbance input"]["coefficient"])

    Ecoupling = transpose([
        1.0 0.0
        0.0 1.0
        0.0 0.0
    ]) # for debug

    # define the parameters struct
    params = SpringMassParams(M, D, K, Ecoupling, Ectrl, Edist)
    # define the state-space model
    simmodel = defmodel(params)

    return (params, simmodel)
end

end
