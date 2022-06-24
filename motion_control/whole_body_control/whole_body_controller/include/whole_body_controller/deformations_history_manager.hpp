#pragma once

#include <Eigen/Core>

#include <string>
#include <vector>



namespace wbc {

/// @class @brief Implements the class that is used to manage the buffer that stores the history of the feet deformations (used to deal with the soft contact model).
class DeformationsHistoryManager {
    public:
        /// @brief Construct a new DeformationsHistoryManager class.
        DeformationsHistoryManager(std::vector<std::string> all_feet_names);

        /// @brief Shrink or expand the vectors representing the history of the deformations when the feet in contact with the terrain change. If a new foot is in contact with the terrain, d_k1 and d_k2 are initialized to a zero vector (for that specific foot).
        /// @param[in] new_contact_feet_names 
        void initialize_deformations_after_planning(std::vector<std::string>& new_contact_feet_names);

        /// @brief Update the history of the feet (desired) deformations after a whole optimization step has been solved.
        /// @param[in] d_k 
        void update_deformations_after_optimization(Eigen::VectorXd& d_k);

        /// @brief Get the deformations history vectors.
        /// @return std::pair<Eigen::VectorXd, Eigen::VectorXd> 
        std::pair<Eigen::VectorXd, Eigen::VectorXd> get_deformations_history();
    private:
        /// @brief The names of all the robot feet.
        std::vector<std::string> all_feet_names;

        /// @brief The names of the robot feet in contact with the terrain.
        std::vector<std::string> contact_feet_names;

        /// @brief The deformations at the previous optimization time step.
        /// @details The deformations must be stored in the same order as how the feet names are stred in all_feet_names (LF -> RF -> LH -> RH).
        Eigen::VectorXd d_k1;
        /// @brief The deformations of two time steps ago.
        /// @details The deformations must be stored in the same order as how the feet names are stred in all_feet_names (LF -> RF -> LH -> RH).
        Eigen::VectorXd d_k2;
};

}