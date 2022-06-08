"""
    SpringMass

submodule that contains all features for spring-mass modeling of flexible appendages of spacecraft
"""
module SpringMass

using LinearAlgebra, StaticArrays
using ..Utilities

export physical2modal, PhysicalSystem, ModalSystem, SpringMassModel, StateSpace, updatestate, modalstate2physicalstate, physicalstate2modalstate, SpringMassParams, defmodel

"""
    PhysicalSystem

Representation of dynamics part of the structural system in physical coordinate.

# Fields

* `dim::Integer`: dimension of the structural system
* `mass_matrix::Matrix`: mass matrix in physical coordinate
* `damping_matrix::Matrix`: damping matrix in physical coordinate
* `stiffness_matrix::Matrix`: stiffness matrix in physical coordinate

# Mathematical representation

This data containter corresponds to the left hand side of the equation of motion as follows:

```math
\\mathbf{M} \\ddot{\\mathbf{x}} + \\mathbf{D} \\dot{\\mathbf{x}} + \\mathbf{K} \\mathbf{x} = \\ldots
```

* ``\\mathbf{M}`` : mass matrix
* ``\\mathbf{D}`` : damping matrix
* ``\\mathbf{K}`` : stiffness matrix

# Constructor

```julia
PhysicalSystem(mass_matrix::Matrix, damping_matrix::Matrix, stiffness_matrix::Matrix)
```

# Example

```julia
# Suppose M, D, K are given

# Create representation of the system in physical coordinate
physicalsystem = PhysicalSystem(M, D, K)
```
"""
struct PhysicalSystem
    dim::Integer

    mass_matrix::Matrix
    damping_matrix::Matrix
    stiffness_matrix::Matrix

    # Constructor
    """
        PhysicalSystem(mass_matrix::Matrix, damping_matrix::Matrix, stiffness_matrix::Matrix)

    Constructor for data container `PhysicalSystem`
    """
    PhysicalSystem(mass_matrix::Matrix, damping_matrix::Matrix, stiffness_matrix::Matrix) = begin
        dim = size(mass_matrix, 1)
        new(dim, mass_matrix, damping_matrix, stiffness_matrix)
    end
end

"""
    ModalSystem

Representation of dynamics part of the structural system in mass-normalized modal coordinate.

# Fields

* `dim::Integer`: dimension of the structural system
* `PHI::Matrix`: modal transformation matrix
* `OMEGA::Matrix`: modal angular velocity matrix
* `XI::Matrix`: modal damping matrix

# Mathematical representation

This data containter corresponds to the left hand side of the equation of motion as follows:

```math
\\ddot{\\mathbf{\\eta}} +  2 \\boldsymbol{\\Xi \\Omega} \\dot{\\mathbf{\\eta}} + \\boldsymbol{\\Omega}^2 \\mathbf{\\eta} = \\ldots
```

The modal transformation is given as:

```math
\\mathbf{x} = \\boldsymbol{\\Phi} \\boldsymbol{\\eta}
```

* ``\\boldsymbol{\\Phi}`` : transformation matrix from modal coordinate to physical coordinate
* ``\\boldsymbol{\\Omega}`` : modal angular velocity matrix
* ``\\boldsymbol{\\Xi}`` : modal damping ratio matrix

# Constructor

```julia
ModalSystem(PHI::Matrix, OMEGA::Matrix, XI::Matrix)
```

# Example

```julia
# You can convert the `physicalsystem::PhysicalSystem` into `::ModalSystem`
physicalsystem = PhysicalSystem(M, C, K)
# Convert representation of the system in modal coordinate
modalsystem = physical2modal(physicalsystem)
```
"""
struct ModalSystem
    dim::Integer

    PHI::Matrix
    OMEGA::Matrix
    XI::Matrix

    # Constructor
    ModalSystem(PHI::Matrix, OMEGA::Matrix, XI::Matrix) = begin
        dim = size(PHI, 1)
        new(dim, PHI, OMEGA, XI)
    end
end

"""
    SpringMassModel

Spring mass representation of the entire system modeling

# Mathematical representation

```math
\\ddot{\\mathbf{\\eta}} + 2 \\boldsymbol{\\Xi \\Omega} \\dot{\\mathbf{\\eta}} + \\boldsymbol{\\Omega}^2 \\mathbf{\\eta} = \\boldsymbol{\\Phi}^{\\mathrm{T}} \\mathbf{D} \\boldsymbol{\\omega} + \\boldsymbol{\\Phi}^{\\mathrm{T}} \\mathbf{F}_{\\mathrm{ctrl}} \\mathbf{f_\\mathrm{ctrl}} + \\boldsymbol{\\Phi}^{\\mathrm{T}} \\mathbf{F}_{\\mathrm{dist}} \\mathbf{f_\\mathrm{dist}}
```

# Fields

* `DOF::Integer`: dimension of the displacement vector of the system
* `dimcontrolinput::Integer`: dimension of the control input vector
* `dimdistinput::Integer`: dimension of the disturbance input vector
* `system::ModalSystem`: mass-normalized modal representation of the system
* `D::AbstractMatrix`: coupling matrix wiht the attitude motion (time derivative of the angular velocity vector)
* `Fctrl::AbstractVecOrMat`: coefficient matrix or vector of the control input vector
* `Fdist::AbstractVecOrMat`: coefficient matrix or vector of the disturbance input vector
"""
struct SpringMassModel
    # degrees of freedom of the system
    DOF::Integer
    # dimension of the control input vector
    dimcontrolinput::Integer
    # dimension of the disturbance input vector
    dimdistinput::Integer
    # Representation of the structural system
    system::ModalSystem

    # coupling matrix with the attitude dynamics (time derivative of the angular velocity vector)
    D::AbstractMatrix

    # control input matrix or vector
    Fctrl::AbstractVecOrMat
    # disturbance input matrix or vector
    Fdist::AbstractVecOrMat

    # Inner constructor for struct `SpringMassModel`
    SpringMassModel(system::ModalSystem, D::AbstractMatrix, Fctrl::AbstractVecOrMat, Fdist::AbstractVecOrMat) = begin
        # get dimension of the system
        DOF = system.dim
        dimcontrolinput = size(Fctrl, 2)
        dimdistinput = size(Fdist, 2)

        new(DOF, dimcontrolinput, dimdistinput, system, D, Fctrl, Fdist)
    end

    SpringMassModel(system::PhysicalSystem, D::AbstractMatrix, Fctrl::AbstractVecOrMat, Fdist::AbstractVecOrMat) = begin

        # convert physical system representation into modal system representation
        system = physical2modal(system)

        # get dimension of the system
        DOF = system.dim
        dimcontrolinput = size(Fctrl, 2)
        dimdistinput = size(Fdist, 2)

        new(DOF, dimcontrolinput, dimdistinput, system, D, Fctrl, Fdist)
    end
end

"""
    StateSpace

State space representation of the structural system. This representation is mainly used for the time evolution of the structural system

# Mathematical representation

State space equation of the structural system with the control input, coupling input (angular velocity of the attitude dynamics), and the disturbance input is given as:

```math
\\frac{d}{dt} \\mathbf{z} = \\mathbf{A} \\mathbf{z} + \\mathbf{B} \\mathbf{u} + \\mathbf{E}_{\\mathrm{cplg}} \\boldsymbol{\\omega} + \\mathbf{E}_{\\mathrm{dist}} \\mathbf{f}_{\\mathrm{dist}}
```

Each matrix is defined as follows:

```math
\\mathbf{A} \\equiv \\begin{bmatrix}
    \\mathbf{0} & \\mathbf{I} \\\\
    - \\boldsymbol{\\Omega}^2 & -2 \\boldsymbol{\\Xi \\Omega}
\\end{bmatrix}
\\\\
\\mathbf{B} \\equiv \\begin{bmatrix}
    \\mathbf{0} \\\\
    \\mathbf{F}_{\\mathrm{ctrl}}
\\end{bmatrix}
\\\\
\\mathbf{E}_{\\mathrm{cplg}} \\equiv \\begin{bmatrix}
    \\mathbf{0} \\\\
    \\mathbf{D}
\\end{bmatrix}
\\\\
\\mathbf{E}_{\\mathrm{dist}} \\equiv \\begin{bmatrix}
    \\mathbf{0} \\\\
    \\mathbf{F}_{\\mathrm{dist}}
\\end{bmatrix}
```

# Fields

* `dimstate::Int`: dimension of the state vector
* `dimctrlinput::Int`: dimension of the control input vector
* `dimdistinput::Int`: dimension of the disturbance input vector
* `sysA::SMatrix`: system matrix
* `sysB::StaticArray`: coefficient matrix or vector for control input
* `sysEcplg::SMatrix`: input matrix for the coupling part (subscript represents coupling)
* `sysEdist::StaticArray`: coefficient matrix or vector for the disturbance input (subscript represents disturbance)

# Constructor

```julia
StateSpace(model::SpringMassModel)
```

"""
struct StateSpace

    # dimension of the state and input vectors
    dimstate::Int
    dimctrlinput::Int
    dimdistinput::Int

    sysA::SMatrix # system matrix
    sysB::StaticArray # control input matrix
    sysEcplg::SMatrix # input matrix for the coupling part (subscript represents coupling)
    sysEdist::StaticArray # input matrix for the disturbance input (subscript represents disturbance)

    modalstate2physicalstate::SMatrix

    StateSpace(model::SpringMassModel) = begin

        dimstate = 2 * model.DOF
        dimctrlinput = model.dimcontrolinput
        dimdistinput = model.dimdistinput

        sysA = SMatrix{dimstate, dimstate}([
            zeros(model.DOF, model.DOF) I
            -model.system.OMEGA^2 -2 * model.system.XI * model.system.OMEGA
        ])

        # Need to switch the definition (`SVector` or `SMatrix`) based on the dimension
        if dimctrlinput == 1
            sysB = SVector{dimstate}([
                zeros(model.DOF)
                model.Fctrl
            ])
        else
            sysB = SMatrix{dimstate, dimctrlinput}([
                zeros(model.DOF, model.dimcontrolinput)
                model.Fctrl
            ])
        end

        sysEcplg = SMatrix{dimstate, 3}([zeros(model.DOF, 3); model.D])

        # Need to switch the definition (`SVector` or `SMatrix`) based on the dimension
        if dimdistinput == 1
            sysEdist = SVector{dimstate}([zeros(model.DOF); model.Fdist])
        else
            sysEdist = SMatrix{dimstate, dimdistinput}([zeros(model.DOF, model.dimdistinput); model.Fdist])
        end

        # transformation matrix for state vector in modal coordinate to physical coordinate
        modalstate2physicalstate = SMatrix{dimstate, dimstate, Real}([
            model.system.PHI zeros(model.DOF, model.DOF)
            zeros(model.DOF, model.DOF) model.system.PHI
        ])

        new(dimstate, dimctrlinput, dimdistinput, sysA, sysB, sysEcplg, sysEdist, modalstate2physicalstate)
    end
end

function physical2modal(physicalsystem::PhysicalSystem)::ModalSystem

    return physical2modal(physicalsystem.mass_matrix, physicalsystem.damping_matrix, physicalsystem.stiffness_matrix)
end

"""
    physical2modal(mass_matrix::Matrix, damping_matrix::Matrix, stiffness_matrix::Matrix)::ModalSystem

return tuple of the modal transformation matrix and modal damping matrix for the mass-normalized modal coordinates
"""
function physical2modal(mass_matrix::Matrix, damping_matrix::Matrix, stiffness_matrix::Matrix)::ModalSystem

    # dimension of the structure
    dim = size(mass_matrix, 1)

    # Eigen value decomposition
    # https://docs.julialang.org/en/v1/stdlib/LinearAlgebra/#LinearAlgebra.eigvecs
    PHI = eigvecs(mass_matrix, stiffness_matrix)

    # mass normalization
    mr = zeros(dim)
    for ind = 1:dim
        mr[ind] = PHI[:, ind]' * mass_matrix * PHI[:, ind]
        PHI[:, ind] = sqrt(1/mr[ind])*PHI[:, ind]

        # redo the calculation of modal mass, mr[idx] == 1.0
        mr[ind] = PHI[:, ind]' * mass_matrix * PHI[:, ind]
    end

    # natural angular frequency matrix
    # OMEGA is defined to be diagonal matrix of natural angular frequency, not its squared value
    OMEGA = sqrt(transpose(PHI) * stiffness_matrix * PHI)

    # calculate modal stiffness matrix Kr = omega^2
    kr = zeros(dim)
    for idx = 1:dim
        kr[idx] = OMEGA[idx, idx]^2
    end

    # modal damping ratio
    cr = zeros(dim)
    xi_r = zeros(dim)
    for idx = 1:dim
        cr[idx] = transpose(PHI[:, idx]) * damping_matrix * PHI[:, idx]
        xi_r[idx] = cr[idx]/(2 * sqrt(mr[idx] * kr[idx]))
    end
    XI = diagm(xi_r)

    # sort matrix with natural angular frequency
    sortidx = sortperm(diag(OMEGA))
    OMEGA = OMEGA[sortidx, sortidx]
    PHI = PHI[:, sortidx]
    XI = XI[sortidx, sortidx]

    return ModalSystem(PHI, OMEGA, XI)
end

"""
    modalstate2physicalstate(model::StateSpace, state::AbstractVector{<:Real})

convert the state vector in modal coordinate into physical coordiniate

# Argument

* `model::StateSpace`: state-space model for simulation
* `state::AbstractVector{<:Real}`: state vector

# Usage

```julia
physicalstate = modalstate2physicalstate(model, state)
```
"""
function modalstate2physicalstate(model::StateSpace, state::AbstractVector{<:Real})
    return model.modalstate2physicalstate * state
end

"""
    modalstate2physicalstate(model::StateSpace, modalstates::AbstractVector{<:AbstractVector})

convert the vector of the state vector in modal coordinate into physical coordiniate

# Argument

* `model::StateSpace`: state-space model for simulation
* `modalstates::AbstractVector{<:AbstractVector}`: vector of state vector, which is a trajectory or time history of the state vector

# Usage

```julia
physicalstates = modalstate2physicalstate(model, states)
```
"""
function modalstate2physicalstate(model::StateSpace, modalstates::AbstractVector{<:AbstractVector})
    datanum = size(modalstates, 1)
    return map(idx -> modalstate2physicalstate(model, modalstates[idx]), 1:datanum)
end

"""
    physicalstate2modalstate(model::StateSpace, physicalstate::AbstractVector{<:Real})

convert the state vector in physical coordinate into modal coordinate

# Argument

* `model::StateSpace`: state-space model for simulation
* `physicalstate::AbstractVector{<:Real}`: state vector
"""
function physicalstate2modalstate(model::StateSpace, physicalstate::AbstractVector{<:Real})
    return inv(model.modalstate2physicalstate) * physicalstate
end

"""
    physicalstate2modalstate(model::StateSpace, physicalstates::AbstractVector{<:AbstractVector})

convert the vector of state vector in physical coordinate into modal coordinate

# Argument

* `model::StateSpace`: state-space model for simulation
* `physicalstates::AbstractVector{<:AbstractVector}`: vector of state vector
"""
function physicalstate2modalstate(model::StateSpace, physicalstates::AbstractVector{<:AbstractVector})
    datanum = size(physicalstates, 1)
    return map(idx -> physicalstate2modalstate(model, physicalstates[idx]), 1:datanum)
end

"""
    updatestate(model::StateSpace, Tsampling::Real, currenttime::Real, currentstate::AbstractVector, angularvelocity::AbstractVector, controlinput::Union{AbstractVector, Real}, distinput::Union{AbstractVector, Real})::AbstractVector

Calculates time evolution of the structural system with Runge-Kutta method
"""
function updatestate(model::StateSpace, Tsampling::Real, currenttime::Real, currentstate::AbstractVector, angularvelocity::AbstractVector, controlinput::Union{AbstractVector, Real}, distinput::Union{AbstractVector, Real})::AbstractVector

    # Runge-Kutta method
    k1 = _calc_differential(model, currenttime              , currentstate                   , angularvelocity, controlinput, distinput)
    k2 = _calc_differential(model, currenttime + Tsampling/2, currentstate + Tsampling/2 * k1, angularvelocity, controlinput, distinput)
    k3 = _calc_differential(model, currenttime + Tsampling/2, currentstate + Tsampling/2 * k2, angularvelocity, controlinput, distinput)
    k4 = _calc_differential(model, currenttime + Tsampling  , currentstate + Tsampling   * k3, angularvelocity, controlinput, distinput)
    nextstate = currentstate + Tsampling/6 *(k1 + 2*k2 + 2*k3 + k4)

    return nextstate
end

function _calc_differential(model::StateSpace, currenttime::Real, currentstate::AbstractVector, angularvelocity::AbstractVector, controlinput::Union{AbstractVector, Real}, distinput::Union{AbstractVector, Real})::AbstractVector

    diff = model.sysA*currentstate + model.sysB*controlinput + model.sysEcplg*angularvelocity + model.sysEdist*distinput

    return diff
end

"""
    SpringMassParams

struct for accomodating the parameters for the spring mass structural model

# Fields

* `M::AbstractMatrix`: mass matrix
* `D::AbstractMatrix`: damping matrix
* `K::AbstractMatrix`: stiffness matrix
* `Ecoupling::AbstractMatrix`: coefficient matrix for the attitude coupling input
* `Econtrol::AbstractVecOrMat`: coefficient matrix for the control input
* `Edisturbance::AbstractVecOrMat`: coefficient matrix for the disturbance input

"""
struct SpringMassParams
    # mass matrix
    M::AbstractMatrix
    # damping matrix
    D::AbstractMatrix
    # stiffness matrix
    K::AbstractMatrix
    # attitude coupling input coefficient matrix
    Ecoupling::AbstractMatrix
    # control input coefficient matrix
    Econtrol::AbstractVecOrMat
    # disturbance input coefficient matrix
    Edisturbance::AbstractVecOrMat
end

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
    M = yamlread2matrix(paramdict["system"]["mass"], (DOF, DOF))
    K = yamlread2matrix(paramdict["system"]["stiffness"], (DOF, DOF))

    if paramdict["system"]["damping"]["config"] == "Rayleigh"
        alpha = paramdict["system"]["damping"]["alpha"]
        beta = paramdict["system"]["damping"]["beta"]
        D = alpha * M + beta * K
    else
        error("damping configuration for \"$(paramdict["system"]["damping"]["config"])\" not found")
    end

    dimcontrolinput = paramdict["control input"]["dimension"]
    Ectrl = yamlread2matrix(paramdict["control input"]["coefficient"], (DOF, dimcontrolinput))

    dimdistinput = paramdict["control input"]["dimension"]
    Edist = yamlread2matrix(paramdict["disturbance input"]["coefficient"], (DOF, dimdistinput))

    Ecoupling = yamlread2matrix(paramdict["coupling"], (DOF, 3))

    # define the parameters struct
    params = SpringMassParams(M, D, K, Ecoupling, Ectrl, Edist)
    # define the state-space model
    simmodel = defmodel(params)

    return (params, simmodel)
end

end
