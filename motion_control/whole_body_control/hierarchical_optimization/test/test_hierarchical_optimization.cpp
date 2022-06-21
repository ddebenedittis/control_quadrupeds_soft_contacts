#include "hierarchical_optimization/hierarchical_qp.hpp"

#include <Eigen/Core>

int main() {
    hopt::HierarchicalQP hqp(2);

    using namespace Eigen;

    MatrixXd A = MatrixXd::Identity(2, 2);
    VectorXd b = VectorXd::Ones(2);
    MatrixXd C = MatrixXd::Identity(2, 2);
    VectorXd d = VectorXd::Ones(2);
    VectorXd we = VectorXd::Ones(2);
    VectorXd wi = VectorXd::Ones(2);

    hqp.SolveQP(A, b, C, d, we, wi, 1);

    return 0;
}