/* ========================================================================== */
/*                                 DESCRIPTION                                */
/* ========================================================================== */

/*

*/



/* ========================================================================== */
/*                              IMPORT LIBRARIES                              */
/* ========================================================================== */

#pragma once

#include <Eigen/Core>



/* ========================================================================== */
/*                                    CODE                                    */
/* ========================================================================== */

namespace hopt {

class HierarchicalQP {
    public:
        /**
         * @brief Construct a new Hierarchical QP object.
         * 
         * @param nTasks the total number of tasks with different priorities of the Hierarchical QP 
         * @param xDim the dimension of the optimization vector
         */
        HierarchicalQP(int n_tasks);

        /**
         * @brief Solve a single task of the hierarchical QP problem.
         * 
         */
        bool SolveQP(
            Eigen::MatrixXd& A,
            Eigen::VectorXd& b,
            Eigen::MatrixXd& C,
            Eigen::VectorXd& d,
            Eigen::VectorXd& we,
            Eigen::VectorXd& wi,
            int Priority
        );
        
    private:
        /**
         * @brief Compute the null space projector of a matrix M.
         * 
         * @param M 
         * @return Eigen::MatrixXd 
         */
        Eigen::MatrixXd NullSpaceProjector(Eigen::MatrixXd M);

        /**
         * @brief Reset the class attributes before starting a new optimization problem.
         * 
         * @param solDim dimension of the optimization vector
         */
        void ResetQP(int sol_dim);

        /// @brief Total number of taks with different priorities */
        int n_tasks_;

        /// @brief Regularization factor introduced in order to ensure that G is Positive Definite. */
        int regularization_ = 1e-6;

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



#include "hierarchical_optimization/hierarchical_qp.tpp"