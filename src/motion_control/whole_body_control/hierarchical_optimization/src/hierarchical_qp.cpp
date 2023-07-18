#include "hierarchical_optimization/hierarchical_qp.hpp"

#include <Eigen/QR>
// #include <Eigen/SVD>
#include "quadprog/quadprog.hpp"

#include <iostream>



namespace hopt {

using namespace Eigen;



/* ========================================================================== */
/*                                   SOLVEQP                                  */
/* ========================================================================== */

/*
    A general task T can be defined as

        [ we * (A x - b)  = v
    T = [
        [ wi * (C x - d) <= w

    where v and w are slack variables.

    Is is formulated as a QP problem

    min_x 1/2 (A x - b)^2 + 1/2 w^2,
    s.t.: C x - d <= w.

    It can be rewritten in the general QP form:

    min_x 1/2 xi^T G xi + g0^T xi
    s.t: CI xi + ci0 >= 0

    where:

    G   =   A^T A
    g0  = - A^T b

    CI  = [ -C, 0 ]
          [  0, I ]
    ci0 = [ d ]
          [ 0 ]

    xi  = [ x ]
          [ w ]

    ============================================================================

    Given a set of tasks T1, ..., Tn, for the task Tp the QP problem becomes:

    G   = [ Zq^T Ap^T Ap Zq, 0 ]
          [               0, I ]
    g0  = [ Zq^T Ap^T (Ap x_opt - bp) ]
          [                         0 ]
    
    CI  = [   0,       I      ]
          [ - C_stack, [0; I] ]
    ci0 = [ 0                                    ]
          [ d - C_stack x_opt + [w_opt_stack; 0] ]
*/

void HierarchicalQP::solve_qp(
    int priority,
    const MatrixXd& A,
    const VectorXd& b,
    const MatrixXd& C,
    const VectorXd& d,
    const VectorXd& we,
    const VectorXd& wi,
    int m_eq
) {
    /* ========================= Weighted A, b, C, d ======================== */

    // Construct the matrices A, b, C, d weighted by we and wi.
    // Each element of we and wi multiplies a whole row of A and C respectively (and an element of b and d). This is more easily implemented using Eigen arrays.

    Eigen::MatrixXd A_w(A.rows(), A.cols());
    Eigen::VectorXd b_w(A.rows());
    Eigen::MatrixXd C_w(C.rows(), A.cols());
    Eigen::VectorXd d_w(C.rows());

    if (A.rows() > 0) {
        A_w = A.array().colwise() * we.array();
        b_w = b.array().colwise() * we.array();
    }
    if (C.rows() > 0) {
        C_w = C.array().colwise() * wi.array();
        d_w = d.array().colwise() * wi.array();
    }


    /* ============================ Solve The QP ============================ */

    solve_qp(
        priority,
        A_w, b_w,
        C_w, d_w,
        m_eq
    );
}



void HierarchicalQP::solve_qp(
    int priority,
    const MatrixXd& A,
    const VectorXd& b,
    const MatrixXd& C,
    const VectorXd& d,
    int m_eq
) {
    /* =================== Setup The Optimization Problem =================== */

    const int A_cols = static_cast<int>(A.cols());    // Dimension of the optimization vector (not counting the slack variables)
    const int C_rows = static_cast<int>(C.rows());    // Number of the inequality constraints of this optimization step

    if (priority == 0) {
        reset_qp(A_cols);
    }
    

    /* ===================== Update C_stack_ And d_stack_ ===================== */

    //           [ C1 ]               [ d1 ]
    // C_stack = [ C2 ]     d_stack = [ d2 ]
    //           [ .. ]               [ .. ]
    //           [ Cp ]               [ dp ]

    if (C_stack_.rows() == 0 && C_rows > 0) {
        C_stack_ = C;
        d_stack_ = d;
    } else if (C_rows > 0) {
        const int C_stack_rows = static_cast<int>(C_stack_.rows());

        C_stack_.conservativeResize(C_stack_rows + C_rows, NoChange);
        C_stack_.bottomRows(C_rows) = C;

        d_stack_.conservativeResize(C_stack_rows + C_rows);
        d_stack_.tail(C_rows) = d;
    }

    const int C_stack_rows = static_cast<int>(C_stack_.rows());


    /* ========================== Compute G And g0 ========================== */

    /*
        G  = [ Zq^T Ap^T Ap Zq, 0
                             0, I ]
        g0 = [ Zq^T Ap^T (Ap x_opt - bp) 
                                       0 ]
    */

    MatrixXd G = MatrixXd::Identity(A_cols + C_rows, A_cols + C_rows);
    VectorXd g0 = VectorXd::Zero(A_cols + C_rows);
    
    if (priority != 0 && A.rows() > 0) {
        G.topLeftCorner(A_cols, A_cols) = Z_.transpose() * A.transpose() * A * Z_;

        g0.head(A_cols) = Z_.transpose() * A.transpose() * (A * sol_ - b);
    }
    else if (priority != 0) {
        G.topLeftCorner(A_cols, A_cols) = MatrixXd::Zero(A_cols, A_cols);

        g0.head(A_cols) = VectorXd::Zero(A_cols);
    } else {
        G.topLeftCorner(A_cols, A_cols) = A.transpose() * A;

        g0.head(A_cols) = - A.transpose() * b;
    }

    // Add the regularization term. This is required in order to ensure that the matrix is positive definite (necessary for the eiquadprog library), and is also desirable.
    G.topLeftCorner(A_cols, A_cols) += regularization_ * MatrixXd::Identity(A_cols, A_cols);


    /* ========================= Compute CI And Ci0 ========================= */

    /*
        CI  = [   0,       I
                - C_stack, [0; I] ]
        ci0 = [ 0       
                d - C_stack x_opt + [w_opt_stack; 0] ]
    */

    MatrixXd CI = MatrixXd::Zero(C_rows + C_stack_rows, A_cols + C_rows);
    CI.topRightCorner(C_rows, C_rows) = MatrixXd::Identity(C_rows, C_rows);
    CI.bottomLeftCorner(C_stack_rows, A_cols) = - C_stack_ * Z_;
    CI.bottomRightCorner(C_rows, C_rows) = MatrixXd::Identity(C_rows, C_rows);

    VectorXd ci0 = VectorXd::Zero(C_rows + C_stack_rows);
    ci0.tail(C_stack_rows) = d_stack_ - C_stack_ * sol_;
    if (priority != 0) {
        ci0.segment(C_rows, C_stack_rows - C_rows) += w_opt_stack_;
    }


    /* ============================ Solve The QP ============================ */

    // Initialize the solution vector
    VectorXd xi_opt(A_cols + C_rows);

    /* In quadprog, the problem is in the following form:
     * min 0.5 * x G x - g0 x
     * s.t.
     * CE.T x - ce0 = 0
     * CI.T x - ci0 >= 0
    */

    // using namespace eiquadprog::solvers;

    // Instantiate the solver object
    // EiquadprogFast qp;
    // qp.reset(A_cols + C_rows, 0, C_rows + C_stack_rows);

    // There are no equality contraints, since the tasks equality constraints are inglobated into the cost function.
    // MatrixXd CE = MatrixXd::Zero(0, A_cols + C_rows);
    // VectorXd ce0 = VectorXd::Zero(0);
    
    // /*EiquadprogFast_status status = */qp.solve_quadprog(
    const int result = solve_quadprog(
        std::move(G),
        - g0,
        CI.transpose(),
        - ci0,
        xi_opt,
        m_eq
    );

    if (result == 1) {
        std::cerr << "At priority " << priority << ", constraints are inconsistent, no solution." << '\n' << std::endl;
    } else if (result == 2) {
        std::cerr << "At priority " << priority << ", matrix G is not positive definite." << '\n' << std::endl;
    }

    // Project the new solution in the null space of the higher priority contraints.
    sol_ += Z_ * xi_opt.head(A_cols);


    /* =========================== Post Processing ========================== */

    // Compute the new null_space_projector for the next time step (if necessary).
    if (priority == 0) {
        // If it is the first task, the computation is slightly easier since Z_ = Identity.
        Z_ = null_space_projector(A);
    } else if (priority < n_tasks_ && A.rows() > 0) {
        // If it is the last task, it is not necessary to compute the null space projector.
        Z_ *= null_space_projector(A * Z_);
    }

    // Update the stack of the w_opt slack variables (only if it is not the last task, and if there are inequality constraints in the current task).
    if (priority < n_tasks_ && C_rows > 0) {
        const int w_opt_stack_rows = static_cast<int>(w_opt_stack_.rows());
        w_opt_stack_.conservativeResize(w_opt_stack_rows + C_rows, NoChange);
        w_opt_stack_.tail(C_rows) = xi_opt.tail(C_rows);
    }
}



/* ========================================================================== */
/*                            NULL_SPACE_PROJECTOR                            */
/* ========================================================================== */

/* ============================== pseudoinverse ============================= */

// Thank you Gael Guennebaud

template<typename MatType>
using PseudoInverseType = Eigen::Matrix<typename MatType::Scalar, MatType::ColsAtCompileTime, MatType::RowsAtCompileTime>;

template<typename MatType>
PseudoInverseType<MatType> pseudoinverse(const MatType &a, double epsilon = std::numeric_limits<double>::epsilon())
{
    using WorkingMatType = Eigen::Matrix<typename MatType::Scalar, Eigen::Dynamic, Eigen::Dynamic, 0,
    MatType::MaxRowsAtCompileTime, MatType::MaxColsAtCompileTime>;
    Eigen::BDCSVD<WorkingMatType> svd(a, Eigen::ComputeThinU | Eigen::ComputeThinV);
    svd.setThreshold(epsilon*std::max(a.cols(), a.rows()));
    Eigen::Index rank = svd.rank();
    Eigen::Matrix<typename MatType::Scalar, Eigen::Dynamic, MatType::RowsAtCompileTime,
    0, Eigen::BDCSVD<WorkingMatType>::MaxDiagSizeAtCompileTime, MatType::MaxRowsAtCompileTime>
    tmp = svd.matrixU().leftCols(rank).adjoint();
    tmp = svd.singularValues().head(rank).asDiagonal().inverse() * tmp;
    return svd.matrixV().leftCols(rank) * tmp;
}

/* ========================== null_space_projector ========================== */

inline MatrixXd HierarchicalQP::null_space_projector(const MatrixXd& M)
{
    // return   Eigen::MatrixXd::Identity(M.cols(), M.cols())
    //        - pseudoinverse(M) * M;

    return   Eigen::MatrixXd::Identity(M.cols(), M.cols())
           - M.completeOrthogonalDecomposition().pseudoInverse().eval() * M;
}



/* ========================================================================== */
/*                                   RESETQP                                  */
/* ========================================================================== */

void HierarchicalQP::reset_qp(int sol_dim)
{
    sol_ = VectorXd::Zero(sol_dim);
    Z_ = MatrixXd::Identity(sol_dim, sol_dim);
    C_stack_ = MatrixXd::Zero(0, sol_dim);
    d_stack_ = VectorXd::Zero(0);
    w_opt_stack_ = VectorXd::Zero(0);
}

} // namespace hopt

