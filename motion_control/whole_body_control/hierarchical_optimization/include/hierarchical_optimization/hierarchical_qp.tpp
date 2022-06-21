/* ========================================================================== */
/*                                 DESCRIPTION                                */
/* ========================================================================== */

/*

*/



/* ========================================================================== */
/*                              IMPORT LIBRARIES                              */
/* ========================================================================== */

#pragma once

#include "hierarchical_optimization/hierarchical_qp.hpp"

#include "eiquadprog/eiquadprog-fast.hpp"

#include <Eigen/Core>
#include <Eigen/QR>
#include <iostream>

// #include "eiquadprog.hpp"





/* ========================================================================== */
/*                        HIERARCHICALQP IMPLEMENTATION                       */
/* ========================================================================== */

namespace hopt {

using namespace Eigen;
using namespace std;



/* ========================================================================== */
/*                              CLASS CONSTRUCTOR                             */
/* ========================================================================== */

HierarchicalQP::HierarchicalQP(int n_tasks)
: n_tasks_(n_tasks) {}



/* ========================================================================== */
/*                                   SOLVEQP                                  */
/* ========================================================================== */

/*
    A general task T can be defined as:

        [ we * (A x - b)  = v
    T = [
        [ wi * (C x - d) <= w

    where v and w are slack variables, formulate is as a QP problem:

    min_x 1/2 (A x - b)^2 + 1/2 w^2
    s.t.: C x - d <= w

    It can be written in the general QP form:

    min_x 1/2 xi^T G xi + g0^T xi
    s.t: CI xi + ci0 >= 0

    where:

    G   = A^T A
    g0  = - A^T b
    CI  = [ -C, 0
             0, I ]
    ci0 = [ d 
            0 ]

    ============================================================================

    Given a set of tasks T1, ..., Tn, for the task Tp the QP problem becomes:

    G = [ Zq^T Ap^T Ap Zq, 0
                        0, I ]
    g0 = [ Zq^T Ap^T (Ap xOpt - bp) 
                                   0 ]
    CI = [   0,      I
           - CStack, [0; I] ]
    ci0 = [ 0       
            d - C_stack_{p-1} xOpt + [wOptStack; 0] ]
*/

bool HierarchicalQP::SolveQP(
    MatrixXd& A,
    VectorXd& b,
    MatrixXd& C,
    VectorXd& d,
    VectorXd& we,
    VectorXd& wi,
    int priority
) {
    /* =================== Setup The Optimization Problem =================== */

    int A_cols = static_cast<int>(A.cols());    // Dimension of the optimization vector (not counting the slack variables)
    int C_rows = static_cast<int>(C.rows());    // Number of the inequality constraints of this optimization step

    if (priority == 1) {
        ResetQP(A_cols);
    }


    /* ===================== Update C_stack_ And d_stack_ ===================== */

    if (priority == 1) {
        C_stack_ = C;
        d_stack_ = d;
    } else {
        int C_stack_rows = static_cast<int>(C_stack_.rows());

        C_stack_.conservativeResize(C_stack_rows + C.rows(), NoChange);
        C_stack_.block(C_stack_rows, 0, C_rows, A_cols) = C;

        d_stack_.conservativeResize(C_stack_rows + d.rows(), NoChange);
        d_stack_.block(C_stack_rows, 0, C_rows,      1) = d;
    }

    int C_stack_rows = static_cast<int>(C_stack_.rows());


    /* ========================== Compute G And g0 ========================== */

    MatrixXd G = MatrixXd::Identity(A_cols + C_rows, A_cols + C_rows);
    VectorXd g0 = VectorXd::Zero(A_cols + C_rows);
    
    if (priority != 1) {
        G.block(0, 0, A_cols, A_cols) = Z_.transpose() * A.transpose() * A * Z_;

        g0.segment(0, A_cols) = Z_.transpose() * A.transpose() * (A * sol_ - b);
    } else {
        G.block(0, 0, A_cols, A_cols) = A.transpose() * A;

        g0.segment(0, A_cols) = - A.transpose() * b;
    }

    // Add the regularization term. This is required in order to ensure that the matrix is positive definite, and is also desirable.
    G.block(0, 0, A_cols, A_cols) += regularization_ * MatrixXd::Identity(A_cols, A_cols);


    /* ========================= Compute CI And Ci0 ========================= */

    MatrixXd CI = MatrixXd::Zero(C_rows + C_stack_rows, A_cols + C_rows);
    CI.block(          0, A_cols,      C_rows, C_rows) = MatrixXd::Identity(C_rows, C_rows);
    CI.block(     C_rows,      0, C_stack_rows, A_cols) = - C_stack_ * Z_;
    CI.block(C_stack_rows, A_cols,      C_rows, C_rows) = MatrixXd::Identity(C_rows, C_rows);

    VectorXd ci0 = VectorXd::Zero(C_rows + C_stack_rows);
    ci0.segment(C_rows, C_stack_rows) = d_stack_ - C_stack_ * sol_;
    if (priority != 1) {
        ci0.segment(C_rows, C_stack_rows - C_rows) = w_opt_stack_;
    }


    /* ============================ Solve The QP ============================ */

    VectorXd xiOpt(A_cols + C_rows);

    /* The problem is in the form:
     * min 0.5 * x G x + g0 x
     * s.t.
     * CE^T x + ce0 = 0
     * CI^T x + ci0 >= 0
    */

    using namespace eiquadprog::solvers;

    EiquadprogFast qp;
    qp.reset(A_cols + C_rows, 0, C_rows + C_stack_rows);

    MatrixXd CE = MatrixXd::Zero(0, 0);
    VectorXd ce0 = VectorXd::Zero(0);

    EiquadprogFast_status status = qp.solve_quadprog(
        G,
        g0,
        CE,
        ce0,
        CI,
        ci0,
        xiOpt
    );

    sol_ += Z_ * xiOpt.segment(0, A_cols);


    /* =========================== Post Processing ========================== */

    if (priority == 1) {
        Z_ = NullSpaceProjector(A);
    } else if (priority < n_tasks_) {
        Z_ *= NullSpaceProjector(A * Z_);
    }

    if (priority < n_tasks_ && C_rows > 0) {
        int w_opt_stack_rows = static_cast<int>(w_opt_stack_.rows());
        w_opt_stack_.conservativeResize(w_opt_stack_rows + C_rows, NoChange);
        w_opt_stack_.segment(w_opt_stack_rows, C_rows) = xiOpt.segment(A_cols, C_rows);
    }

    return 0;

}



/* ========================================================================== */
/*                             NULLSPACEPROJECTOR                             */
/* ========================================================================== */

MatrixXd HierarchicalQP::NullSpaceProjector(MatrixXd M) {
    return M.completeOrthogonalDecomposition().pseudoInverse();
}



/* ========================================================================== */
/*                                   RESETQP                                  */
/* ========================================================================== */

void HierarchicalQP::ResetQP(int sol_dim) {
    sol_ = VectorXd::Zero(sol_dim);
    Z_ = MatrixXd::Identity(sol_dim, sol_dim);
    C_stack_ = MatrixXd::Zero(0, sol_dim);
    d_stack_ = VectorXd::Zero(0);
    w_opt_stack_ = VectorXd::Zero(0);
}

} // namespace hopt

