#include "hierarchical_optimization/hierarchical_qp.hpp"

#include <Eigen/Core>

#include <iostream>

int main()
{
    hopt::HierarchicalQP hqp(4);

    using namespace Eigen;

    MatrixXd A = MatrixXd::Identity(2, 4);
    VectorXd b = VectorXd::Ones(2);
    MatrixXd C = MatrixXd::Identity(2, 4);
    VectorXd d = VectorXd::Ones(2);
    VectorXd we = VectorXd::Ones(2);
    VectorXd wi = VectorXd::Ones(2);

    hqp.solve_qp(A, b, C, d, we, wi, 0);

    std::cout << hqp.get_sol() << std::endl;

    MatrixXd A2(1,4); A2 << 1, 3, 4, 6;
    VectorXd b2(1); b2 << 99;
    MatrixXd C2(2, 4); C2 << 4, 3, 5, 7,
                             0, 0, 0, 1;
    VectorXd d2(2); d2 << 3,
                          3;
    VectorXd we2 = VectorXd::Ones(1);
    VectorXd wi2 = VectorXd::Ones(2);

    hqp.solve_qp(A2, b2, C2, d2, we2, wi2, 1);

    std::cout << hqp.get_sol() << std::endl;

    return 0;
}