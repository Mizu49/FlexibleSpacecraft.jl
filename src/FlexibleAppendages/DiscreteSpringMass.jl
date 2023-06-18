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
struct SpringMassParams{DOF, dimctrl, dimdist}<:AppendagesBase.AbstractAppendageParameters
    # mass matrix
    M::SMatrix{DOF, DOF}
    # damping matrix
    D::SMatrix{DOF, DOF}
    # stiffness matrix
    K::SMatrix{DOF, DOF}
    # control input coefficient matrix
    Econtrol::SMatrix{DOF, dimctrl}
    # disturbance input coefficient matrix
    Edisturbance::SMatrix{DOF, dimdist}
end

function SpringMassParams(
    M::AbstractMatrix,
    D::AbstractMatrix,
    K::AbstractMatrix,
    Econtrol::AbstractMatrix,
    Edisturbance::AbstractMatrix,
    )
    DOF = size(M, 1)
    dimctrl = size(Econtrol, 2)
    dimdist = size(Edisturbance, 2)
    return SpringMassParams{DOF, dimctrl, dimdist}(M, D, K, Econtrol, Edisturbance)
end

"""
    PhysicalSystem

Representation of dynamics part of the structural system in physical coordinate.

# Fields

* `DOF::Integer`: dimension of the structural system
* `M::Matrix`: mass matrix in physical coordinate
* `D::Matrix`: damping matrix in physical coordinate
* `K::Matrix`: stiffness matrix in physical coordinate

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
PhysicalSystem(M::Matrix, D::Matrix, K::Matrix)
```

# Example

```julia
# Suppose M, D, K are given

# Create representation of the system in physical coordinate
physicalsystem = PhysicalSystem(M, D, K)
```
"""
struct PhysicalSystem
    DOF::Integer
    dimctrl::Integer
    dimdist::Integer
    M::SMatrix
    D::SMatrix
    K::SMatrix
    Ectrl::SMatrix
    Edist::SMatrix
end

function PhysicalSystem(
    M::SMatrix,
    D::SMatrix,
    K::SMatrix,
    Ectrl::SMatrix,
    Edist::SMatrix
    )
    DOF = size(M, 1)
    dimctrl = size(Ectrl, 2)
    dimdist = size(Edist, 2)
    return PhysicalSystem(DOF, dimctrl, dimdist, M, D, K, Ectrl, Edist)
end

function PhysicalSystem(
    M::Matrix,
    D::Matrix,
    K::Matrix,
    Ectrl::Matrix,
    Edist::Matrix
    )
    DOF = size(M, 1)
    dimctrl = size(Ectrl, 2)
    dimdist = size(Edist, 2)
    return PhysicalSystem(
        DOF,
        dimctrl,
        dimdist,
        SMatrix{DOF, DOF}(M),
        SMatrix{DOF, DOF}(D),
        SMatrix{DOF, DOF}(K),
        SMatrix{DOF, dimctrl}(Ectrl),
        SMatrix{DOF, dimdist}(Edist)
    )
end

function PhysicalSystem(params::SpringMassParams{DOF, dimctrl, dimdist}) where {DOF, dimctrl, dimdist}
    return PhysicalSystem(DOF, dimctrl, dimdist, params.M, params.D, params.K, params.Econtrol, params.Edisturbance)
end

"""
    ModalSystem

Representation of dynamics part of the structural system in mass-normalized modal coordinate.

# Fields

* `DOF::Integer`: dimension of the structural system
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
    dimmode::Integer
    DOF::Integer
    dimctrl::Integer
    dimdist::Integer
    PHI::SMatrix
    OMEGA::SMatrix
    XI::SMatrix
end

function ModalSystem(PHI::Matrix, OMEGA::Matrix, XI::Matrix, Ectrl::Matrix, Edist::Matrix)
    DOF = size(PHI, 1)
    dimmode = size(PHI, 2)
    dimctrl = size(Ectrl, 2)
    dimdist = size(Edist, 2)
    return ModalSystem(
        dimmode,
        DOF,
        dimctrl,
        dimdist,
        SMatrix{DOF, DOF}(PHI),
        SMatrix{DOF, DOF}(OMEGA),
        SMatrix{DOF, DOF}(XI)
    )
end

function ModalSystem(PHI::SMatrix, OMEGA::SMatrix, XI::SMatrix, Ectrl::SMatrix, Edist::SMatrix)
    DOF = size(PHI, 1)
    dimmode = size(PHI, 2)
    dimctrl = size(Ectrl, 2)
    dimdist = size(Edist, 2)
    return ModalSystem(dimmode, DOF, dimctrl, dimdist, PHI, OMEGA, XI)
end


"""
    physical2modal(M::Matrix, D::Matrix, K::Matrix)::ModalSystem

return tuple of the modal transformation matrix and modal damping matrix for the mass-normalized modal coordinates
"""
function physical2modal(physicalsystem::PhysicalSystem)::ModalSystem

    M = physicalsystem.M
    K = physicalsystem.K
    D = physicalsystem.D
    DOF = physicalsystem.DOF

    #TODO: fix this to implement modal reduction feature
    dimmode = DOF

    # Eigen value decomposition
    # https://docs.julialang.org/en/v1/stdlib/LinearAlgebra/#LinearAlgebra.eigvecs
    PHI = eigvecs(M, K)

    # mass normalization
    mr = zeros(DOF)
    for ind = 1:DOF
        mr[ind] = PHI[:, ind]' * M * PHI[:, ind]
        PHI[:, ind] = sqrt(1/mr[ind])*PHI[:, ind]

        # redo the calculation of modal mass, mr[idx] == 1.0
        mr[ind] = PHI[:, ind]' * M * PHI[:, ind]
    end

    # natural angular frequency matrix
    # OMEGA is defined to be diagonal matrix of natural angular frequency, not its squared value
    OMEGA = sqrt(transpose(PHI) * K * PHI)

    # calculate modal stiffness matrix Kr = omega^2
    kr = zeros(DOF)
    for idx = 1:DOF
        kr[idx] = OMEGA[idx, idx]^2
    end

    # modal damping ratio
    cr = zeros(DOF)
    xi_r = zeros(DOF)
    for idx = 1:DOF
        cr[idx] = transpose(PHI[:, idx]) * D * PHI[:, idx]
        xi_r[idx] = cr[idx]/(2 * sqrt(mr[idx] * kr[idx]))
    end
    XI = diagm(xi_r)

    # sort matrix with natural angular frequency
    sortidx = sortperm(diag(OMEGA))
    OMEGA = OMEGA[sortidx, sortidx]
    PHI = PHI[:, sortidx]
    XI = XI[sortidx, sortidx]

    return ModalSystem(
        dimmode,
        DOF,
        physicalsystem.dimctrl,
        physicalsystem.dimdist,
        SMatrix{DOF, dimmode}(PHI),
        SMatrix{dimmode, dimmode}(OMEGA),
        SMatrix{dimmode, dimmode}(XI)
    )
end
