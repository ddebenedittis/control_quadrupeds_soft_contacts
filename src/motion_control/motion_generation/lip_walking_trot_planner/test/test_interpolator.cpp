#include "lip_walking_trot_planner/interpolator.hpp"

#include <gtest/gtest.h>

#include <cstdlib>
#include <iostream>



using namespace Eigen;



inline void test_eq_vectors(const VectorXd& v1, const VectorXd& v2)
{
     EXPECT_EQ(v1.size(), v2.size()) << "The solution has wrong dimension";

     for (uint i = 0; i < static_cast<uint>(v1.size()); i++) {
          EXPECT_TRUE((std::abs(v1[i] - v2[i]) < 1e-5)) << "The solution is wrong at index " << i;
     }
}

inline void test_eq_tuple(
    const std::tuple<Vector3d, Vector3d, Vector3d>& t1,
    const std::tuple<Vector3d, Vector3d, Vector3d>& t2
) {
    test_eq_vectors(std::get<0>(t1), std::get<0>(t2));
    test_eq_vectors(std::get<1>(t1), std::get<1>(t2));
    test_eq_vectors(std::get<2>(t1), std::get<2>(t2));
}



TEST(lip_walking_trot_planner, interpolator_test)
{
    auto interp = lip_walking_trot_planner::Interpolator();

    /* ============================= Spline_5th ============================= */

    auto method = lip_walking_trot_planner::InterpolationMethod::Spline_5th;
    interp.set_method(method);

    Vector3d init_pos = {0, 0, 0};
    Vector3d end_pos = {1, 0, - interp.get_foot_penetration()};

    Vector3d zero = {0, 0, 0};

    test_eq_tuple(
        std::make_tuple(init_pos, zero, zero),
        interp.interpolate(init_pos, end_pos, 0)
    );
    test_eq_tuple(
        std::make_tuple(end_pos, zero, zero),
        interp.interpolate(init_pos, end_pos, 1)
    );
    test_eq_vectors(
        (Vector3d() << 0.5, 0, interp.get_step_height()).finished(),
        std::get<0>(interp.interpolate(init_pos, end_pos, 0.5))
    );
}



int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
