#include <whole_body_controller/deformations_history_manager.hpp>

#include <algorithm>



namespace wbc {

/* ========================================================================== */
/*                   DEFORMATIONSHISTORYMANAGER CONSTRUCTOR                   */
/* ========================================================================== */

DeformationsHistoryManager::DeformationsHistoryManager(const std::vector<std::string>& all_feet_names)
: all_feet_names(all_feet_names) {}



/* ========================================================================== */
/*                   INITIALIZE_DEFORMATIONS_AFTER_PLANNING                   */
/* ========================================================================== */

void DeformationsHistoryManager::initialize_deformations_after_planning(const std::vector<std::string>& new_contact_feet_names)
{
    // If the current feet in contact with the terrain are the same as in the previous time step, nothing has to be done and the function can return
    
    if (new_contact_feet_names.size() == contact_feet_names.size()) {
        for (int i = 0; i < static_cast<int>(new_contact_feet_names.size()); i++) {
            if (new_contact_feet_names[i] != contact_feet_names[i]) {
                break;
            }
            if (i == static_cast<int>(new_contact_feet_names.size()) - 1) {
                return;
            }
        }
    }


    /* ====================================================================== */

    // Initialize the new deformations vectors.
    Eigen::VectorXd d_k1_temp(def_size * new_contact_feet_names.size());
    Eigen::VectorXd d_k2_temp(def_size * new_contact_feet_names.size());

    // Populate the new deformations vectors.
    for (int i = 0; i < static_cast<int>(new_contact_feet_names.size()); i++) {
        auto iterator = std::find(contact_feet_names.begin(), contact_feet_names.end(), new_contact_feet_names[i]);

        if (iterator == contact_feet_names.end()) {
            // This foot has just entered in contact with the terrain, initialize its deformations to zero.

            d_k1_temp.segment(def_size*i, def_size).setZero();
            d_k2_temp.segment(def_size*i, def_size).setZero();
        } else {
            // This foot was already in contact with the terrain, maintain the history of its deformations.

            int index = std::distance(contact_feet_names.begin(), iterator);
            
            d_k1_temp.segment(def_size*i, def_size) = d_k1.segment(def_size*index, def_size);
            d_k2_temp.segment(def_size*i, def_size) = d_k2.segment(def_size*index, def_size);
        }
    }

    // Save the new contact_feet_names and the new deformations history vectors
    contact_feet_names = new_contact_feet_names;
    d_k1 = d_k1_temp;
    d_k2 = d_k2_temp;

    return;
}


/* ========================================================================== */
/*                   UPDATE_DEFORMATIONS_AFTER_OPTIMIZATION                   */
/* ========================================================================== */

void DeformationsHistoryManager::update_deformations_after_optimization(const Eigen::Ref<const Eigen::VectorXd> d_k)
{
    d_k2 = d_k1;
    d_k1 = d_k;
}



/* ========================================================================== */
/*                          GET_DEFORMATIONS_HISTORY                          */
/* ========================================================================== */

std::pair<Eigen::VectorXd, Eigen::VectorXd> DeformationsHistoryManager::get_deformations_history()
{
    return std::make_pair(d_k1, d_k2);
}

}