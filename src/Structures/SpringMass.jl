"""
    SpringMass

submodule that contains all features for spring-mass modeling of flexible appendages of spacecraft
"""
module SpringMass

using LinearAlgebra, StaticArrays

"""
    PhysicalSystem

Representation of the structural system in physical coordinate

## Fields

`mass_matrix::AbstractMatrix`: mass matrix in physical coordinate
`damping_matrix::AbstractMatrix`: damping matrix in physical coordinate
`stiffness_matrix::AbstractMatrix`: stiffness matrix in physical coordinate
"""
struct PhysicalSystem
    dim::Integer

    mass_matrix::AbstractMatrix
    damping_matrix::AbstractMatrix
    stiffness_matrix::AbstractMatrix

    # Constructor
    PhysicalSystem(mass_matrix::AbstractMatrix, damping_matrix::AbstractMatrix, stiffness_matrix::AbstractMatrix) = begin
        dim = size(mass_matrix, 1)
        # convert to SMatrix for fast computation
        mass_matrix = SMatrix{dim, dim}(mass_matrix)
        damping_matrix = SMatrix{dim, dim}(damping_matrix)
        stiffness_matrix = SMatrix{dim, dim}(stiffness_matrix)

        new(dim, mass_matrix, damping_matrix, stiffness_matrix)
    end
end

struct SpringMassModel
    # degrees of freedom (DOF) of the structure
    DOF
    # dimension of the control input vector
    dimcontrolinput
    # dimension of the disturbance input vector
    dimdistinput
    # transformation matrix. i.e. x = 𝚽𝛈, modal coordinates are mass-normalized
    PHI
    # modal damping matrix
    XI
    # coupling matrix with the attitude dynamics (time derivative of the angular velocity vector)
    D
    # disturbance input matrix
    F
end

struct StateSpace
    sysA # system matrix
    sysB # control input matrix
    sysEc # input matrix for the coupling part (subscript c represents coupling)
    sysEd # input matrix for the disturbance input (subscript d represents disturbance)

    StateSpace(model::SpringMassModel) = begin
        sysA = [
            zeros(model.DOF) I
            -PHI^2 -2*XI*PHI
        ]
        sysB = [
            zeros(model.DOF, model.dimcontrolinput)
            model.controlinputmatrix
        ]
        sysEc = [zeros(model.DOF, 3); D]
        sysEd = [zeros(model.DOF, model.dimdistinput); model.F]
    end
end

"""
    _mode_decomposition(mass_matrix::AbstractMatrix, damping_matrix::AbstractMatrix, stiffness_matrix::AbstractMatrix)::Tuple

return tuple of the modal transformation matrix and modal damping matrix for the mass-normalized modal coordinates
"""
function _mode_decomposition(mass_matrix::AbstractMatrix, damping_matrix::AbstractMatrix, stiffness_matrix::AbstractMatrix)::Tuple

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

    return (PHI, OMEGA, XI)
end

end
