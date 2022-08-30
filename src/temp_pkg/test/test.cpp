#include <Eigen/Core>
// #include "eiquadprog/eiquadprog-fast.hpp"
#include "temp_pkg/solve.QP.c"
#include <vector>
#include <iostream>
#include <fstream>
#include <algorithm>

using namespace Eigen;

template<typename M>
M load_csv (const std::string & path) {
    std::ifstream indata;
    indata.open(path);
    std::string line;
    std::vector<double> values;
    uint rows = 0;
    while (std::getline(indata, line)) {
        std::stringstream lineStream(line);
        std::string cell;
        while (std::getline(lineStream, cell, ',')) {
            values.push_back(std::stod(cell));
        }
        ++rows;
    }
    return Map<const Matrix<typename M::Scalar, M::RowsAtCompileTime, M::ColsAtCompileTime, RowMajor>>(values.data(), rows, values.size()/rows);
}

int main()
{
    MatrixXd G_c = load_csv<MatrixXd>("G-C++.csv");
    VectorXd g0_c = load_csv<MatrixXd>("g0-C++.csv");
    MatrixXd CI_c = load_csv<MatrixXd>("CI-C++.csv");
    VectorXd ci0_c = load_csv<MatrixXd>("ci0-C++.csv");

    VectorXd xi_opt(G_c.rows());
    xi_opt.setZero();

    // using namespace eiquadprog::solvers;

    // // Instantiate the solver object
    // EiquadprogFast qp;
    // qp.reset(G_c.rows(), 0, 5);

    // There are no equality contraints, since the tasks equality constraints are inglobated into the cost function.
    // MatrixXd CE_c = MatrixXd::Zero(0, G_c.rows());
    // VectorXd ce0_c = VectorXd::Zero(0);

    // CI_c = MatrixXd::Zero(4, G_c.rows());
    // ci0_c = -100 * VectorXd::Ones(4);

    // CI_c(0,0) = CI_c(1,1) = CI_c(2,2) = CI_c(3,3) = 1.;

    // EiquadprogFast_status status = qp.solve_quadprog(
    //     G_c,
    //     g0_c,
    //     CE_c,
    //     ce0_c,
    //     CI_c,
    //     ci0_c,
    //     xi_opt
    // );

    g0_c = -g0_c;
    CI_c.transposeInPlace();
    ci0_c = -ci0_c;

    int n = G_c.rows();
    int q = CI_c.cols();

    int meq = 0;
    int factorized = 0;
    double obj;
    
    std::vector<double> x(n);
    std::vector<double> lagr(q);
    std::vector<int> iact(q);
    int nact;
    std::vector<int> iter(2);
    std::vector<double> work(2*n + 2*q + std::min(n,q)*(std::min(n,q)+5)/2);

    int result;

    result = qpgen2_(
        G_c.data(), g0_c.data(), n,
        x.data(), &lagr[0], &obj,
        CI_c.data(), ci0_c.data(), q, meq,
        &iact[0], &nact, &iter[0],
        &work[0], factorized
    );

    xi_opt = Eigen::Map<MatrixXd>(&x[0], n-1, 1);

    if(result!=0) {
        std::cout << result <<std::endl;
    }

    std::cout << "xi_opt:\n" << xi_opt << "\n" << std::endl;
}