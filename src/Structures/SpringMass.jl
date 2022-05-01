"""
    SpringMass

submodule that contains all features for spring-mass modeling of flexible appendages of spacecraft
"""
module SpringMass

using LinearAlgebra, StaticArrays

export physical2modal, PhysicalSystem, ModalSystem, SpringMassModel, StateSpace, updatestate, modalstate2physicalstate, physicalstate2modalstate

"""
    PhysicalSystem

Representation of the structural system in physical coordinate

## Fields

* `dim::Integer`: dimension of the structural system
* `mass_matrix::Matrix`: mass matrix in physical coordinate
* `damping_matrix::Matrix`: damping matrix in physical coordinate
* `stiffness_matrix::Matrix`: stiffness matrix in physical coordinate
"""
struct PhysicalSystem
    dim::Integer

    mass_matrix::Matrix
    damping_matrix::Matrix
    stiffness_matrix::Matrix

    # Constructor
    PhysicalSystem(mass_matrix::Matrix, damping_matrix::Matrix, stiffness_matrix::Matrix) = begin
        dim = size(mass_matrix, 1)
        new(dim, mass_matrix, damping_matrix, stiffness_matrix)
    end
end

"""
    ModalSystem

Representation of the structural system in modal coordinate

## Fields

* `dim::Integer`: dimension of the structural system
* `PHI::Matrix`: modal transformation matrix
* `OMEGA::Matrix`: modal angular velocity matrix
* `XI::Matrix`: modal damping matrix
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

```math
\\ddot{\\mathbf{\\eta}} \\eta + 2 \\boldsymbol{\\Xi \\Omega} \\dot{\\mathbf{\\eta}} + \\boldsymbol{\\Omega}^2 \\mathbf{\\eta}
```

## Members

* `DOF::Integer`: dimension of the displacement vector of the system
* `dimcontrolinput::Integer`: dimension of the control input vector
* `dimdistinput::Integer`: dimension of the disturbance input vector
* `system::ModalSystem`: mass-normalized modal representation of the system
* `D::AbstractMatrix`: coupling matrix wiht the attitude motion (time derivative of the angular velocity vector)
* `Fctrl::AbstractArray`: coefficient matrix or vector of the control input vector
* `Fdist::AbstractArray`: coefficient matrix or vector of the disturbance input vector
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

    # control input matrix
    Fctrl::AbstractArray
    # disturbance input matrix
    Fdist::AbstractArray

    # Inner constructor for struct `SpringMassModel`
    SpringMassModel(system::ModalSystem, D::AbstractMatrix, Fctrl::AbstractArray, Fdist::AbstractArray) = begin
        # get dimension of the system
        DOF = system.dim
        dimcontrolinput = size(Fctrl, 2)
        dimdistinput = size(Fdist, 2)

        new(DOF, dimcontrolinput, dimdistinput, system, D, Fctrl, Fdist)
    end

    SpringMassModel(system::PhysicalSystem, D::AbstractMatrix, Fctrl::AbstractArray, Fdist::AbstractArray) = begin

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
            -model.system.OMEGA^2 -2 * model.system.XI * model.system.PHI
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

end
