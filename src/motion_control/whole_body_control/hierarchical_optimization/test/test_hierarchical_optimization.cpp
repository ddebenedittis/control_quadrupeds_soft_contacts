#include "hierarchical_optimization/hierarchical_qp.hpp"

#include <gtest/gtest.h>

#include <cstdlib>
#include <iostream>


using namespace Eigen;

inline void test_equal_vectors(const VectorXd& v1, const VectorXd& v2)
{
     EXPECT_EQ(v1.size(), v2.size()) << "The solution has wrong dimension";

     for (uint i = 0; i < static_cast<uint>(v1.size()); i++) {
          EXPECT_TRUE((std::abs(v1[i] - v2[i]) < 1e-5)) << "The solution is wrong at index " << i;
     }
}



TEST(hierarchical_optimization, correct_solution)
{
    hopt::HierarchicalQP hqp(4);

    MatrixXd A = MatrixXd::Zero(4, 6);
    VectorXd b = VectorXd::Zero(4);
    MatrixXd C = MatrixXd::Zero(2, 6);
    VectorXd d = VectorXd::Zero(2);
    VectorXd we = VectorXd::Ones(4);
    VectorXd wi = VectorXd::Ones(2);

    A << 0.4387,   0.1869,   0.7094,   0.6551,   0.9597,   0.7513,
         0.3816,   0.4898,   0.7547,   0.1626,   0.3404,   0.2551,
         0.7655,   0.4456,   0.2760,   0.1190,   0.5853,   0.5060,
         0.7952,   0.6463,   0.6797,   0.4984,   0.2238,   0.6991;

    b << 0.6948,   0.3171,   0.9502,   0.0344;

    C << 0.8909,   0.5472,   0.1493,   0.8407,   0.8143,   0.9293,
         0.9593,   0.1386,   0.2575,   0.2543,   0.2435,   0.3500;

    d << 0.3804,    0.0759;

    hqp.solve_qp(0, A, b, C, d, we, wi);

    VectorXd sol(6);
    sol << -0.181384, 0.117233, 0.343859, 0.142011, 0.356041, 0.156563;

    A.resize(1,6);
    A << 0.1622,   0.7943,   0.3112,   0.5285,   0.1656,   0.6020;

    b.resize(1);
    b << 0;

    C.resize(2,6);
    C << 0.2630,   0.6892,   0.4505,   0.2290,   0.1524,   0.5383,
         0.6541,   0.7482,   0.0838,   0.9133,   0.8258,   0.9961;

    d.resize(2);
    d << 0.0782, 0.0782;

    we = VectorXd::Ones(1);
    wi = VectorXd::Ones(2);

    hqp.solve_qp(1, A, b, C, d, we, wi);

    std::cout << hqp.get_sol() << std::endl;

    sol << -0.166568, -0.156958, 0.0831332, -2.1497, 1.06716, 1.49388;
    test_equal_vectors(hqp.get_sol(), sol);
}



int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
