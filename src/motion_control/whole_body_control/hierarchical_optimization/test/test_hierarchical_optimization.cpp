#include "hierarchical_optimization/hierarchical_qp.hpp"

#include <iostream>

int main()
{
    hopt::HierarchicalQP hqp(4);

    using namespace Eigen;

    MatrixXd A = MatrixXd::Zero(2, 10);
    VectorXd b = VectorXd::Zero(2);
    MatrixXd C = MatrixXd::Zero(2, 10);
    VectorXd d = VectorXd::Zero(2);
    VectorXd we = VectorXd::Ones(2);
    VectorXd wi = VectorXd::Ones(2);

    A(1,9) = 13; A(0,4) = 8; A(0, 0) = 43;
    b(0) = 3;
    b(1) = 12;
    C(1, 8) = 8; C(1, 4) = 90; C(0,5) = 1.5;
    d(0) = 1; d(1) = 1;

    hqp.solve_qp(0, A, b, C, d, we, wi);

    std::cout << hqp.get_sol() << "\n\n" << std::endl;

    MatrixXd A2 = MatrixXd::Zero(1, 10);
    VectorXd b2 = VectorXd::Zero(1);
    MatrixXd C2 = MatrixXd::Zero(3,10);
    VectorXd d2 = VectorXd::Zero(3);
    VectorXd we2 = VectorXd::Ones(1);
    VectorXd wi2 = VectorXd::Ones(3);

    A2(0, 2) = 6; A2(0, 7) = 7; A2(0,1) = 4.3232;
    C2(0,0) = 1; C2(0,1) = 1;
    C2(1,4) = 0.3; C2(1,8) = 0.8; C2(1,9) = 0.99;
    C2(2, 7) = 8;
    d2(0) = -1;
    d2(1) = 11;
    d2(2) = -9;

    hqp.solve_qp(1, A2, b2, C2, d2, we2, wi2);

    std::cout << hqp.get_sol() << std::endl;

    // MatrixXd A = MatrixXd::Ones(1,2);
    // VectorXd b = VectorXd::Ones(1);
    // MatrixXd C = MatrixXd::Zero(1,2); C(0,0) = 0.5;
    // VectorXd d = VectorXd::Ones(1); d(0) = -12;
    // VectorXd we = VectorXd::Ones(1);
    // VectorXd wi = VectorXd::Ones(1);

    // hqp.solve_qp(0, A, b, C, d, we, wi);

    // std::cout << hqp.get_sol() << "\n\n" << std::endl;

    return 0;
}