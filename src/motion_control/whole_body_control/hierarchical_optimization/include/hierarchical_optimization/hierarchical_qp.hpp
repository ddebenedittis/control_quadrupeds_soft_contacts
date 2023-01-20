#pragma once

#include <Eigen/Core>



namespace hopt {

/// @class @brief 
class HierarchicalQP {
public:
    /// @brief Construct a new Hierarchical QP object.
    /// @param[in] nTasks the total number of tasks with different priorities of the Hierarchical QP 
    HierarchicalQP(int n_tasks);

    /// @brief Solve a single prioritized task of the hierarchical QP problem.
    void solve_qp(
        int priority,
        const Eigen::MatrixXd& A,
        const Eigen::VectorXd& b,
        const Eigen::MatrixXd& C,
        const Eigen::VectorXd& d,
        const Eigen::VectorXd& we,
        const Eigen::VectorXd& wi
    );

    /// @brief Get the QP problem solution
    Eigen::VectorXd get_sol() { return sol_; }
    
private:
    /// @brief Compute the null space projector of a matrix M.
    /// @param[in] M 
    /// @return Eigen::MatrixXd Square matrix of dimension (A_cols, A_cols)
    Eigen::MatrixXd null_space_projector(Eigen::MatrixXd M);

    /// @brief Reset the class attributes before starting a new optimization problem.
    /// @param[in] solDim dimension of the optimization vector
    void reset_qp(int sol_dim);

    /* ================================================================== */

    /// @brief Total number of tasks with different priorities */
    int n_tasks_;

    /// @brief Regularization factor introduced in order to ensure that G is Positive Definite. */
    double regularization_ = 1e-6;

    /// @brief Optimization vector. */
    Eigen::VectorXd sol_;

    /// @brief Null Space basis of the stack of equality constraints. */
    Eigen::MatrixXd Z_;

    /// @brief Stack of the inequality constraints matrices Cp. */
    Eigen::MatrixXd C_stack_;
    /// @brief Stack of the inequality constraints vectord dp. */
    Eigen::VectorXd d_stack_;
    /// @brief Stack of the optimal slack variables wOpt (wi * (C x - d) <= w). */
    Eigen::VectorXd w_opt_stack_;
};

} // namespace hopt