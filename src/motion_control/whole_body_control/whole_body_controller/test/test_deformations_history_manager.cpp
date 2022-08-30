#include "whole_body_controller/deformations_history_manager.hpp"

#include <iostream>


int main()
{
    using namespace wbc;
    using namespace std;

    std::vector<std::string> all_feet_names {"LF", "RF", "LF", "RH"};

    DeformationsHistoryManager def(all_feet_names);

    std::vector<std::string> contact_feet {"RF"};

    def.initialize_deformations_after_planning(contact_feet);

    auto t = def.get_deformations_history();

    cout << t.first << std::endl;
    cout << "\n";

    contact_feet = {"LF", "RH"};

    def.initialize_deformations_after_planning(contact_feet);

    t = def.get_deformations_history();

    cout << t.first << std::endl;
    cout << "\n";

    Eigen::VectorXd d_k(6);
    d_k << 1,2,3, 4,5,6;

    def.update_deformations_after_optimization(d_k);

    t = def.get_deformations_history();

    cout << t.first << std::endl;
    cout << "\n";

    contact_feet = {"RF", "RH"};

    def.initialize_deformations_after_planning(contact_feet);

    t = def.get_deformations_history();

    cout << t.first << std::endl;
    cout << "\n";

    return 0;
}