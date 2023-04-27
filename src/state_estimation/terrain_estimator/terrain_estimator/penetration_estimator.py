import numpy as np

from gazebo_msgs.msg import ContactsState



class PenetrationEstimator():
    """
    Estimate the maximum penetration during contact with the terrain.
    The maximum penetration returned is the maximum between the current and the previous contact phase.
    """

    def __init__(self) -> None:
        # Order: LF RF LH RH
        
        self.touchdown_height = np.full([4], np.nan)        # nan when not in contact
        self.maximum_penetration = np.zeros(4)
        self.maximum_penetration_previous_step = np.zeros(4)
       
        
    def update_penetration_i(self, msg: ContactsState, index: int) -> None:
        """
        Update the internal state upon receiving the contact information.

        Args:
            msg (ContactsState): index-th foot contact state (contact position, if in contact)
            index (int): index identifying the foot (1 = LF, 2 = RF, 3 = LH, 4 = RH)
        """
        
        if len(msg.states) == 0:
            # Set the touchdown height to nan when not in contact.
            self.touchdown_height[index] = np.nan
            
        elif np.isnan(self.touchdown_height[index]):
            # The foot just entered the contact phase with the terrain.
            
            # Initialize the touchdown height.
            self.touchdown_height[index] = msg.states[0].contact_positions[0].z
            # Update the previous maximum penetration.
            self.maximum_penetration_previous_step[index] = self.maximum_penetration[index]
            # Reset the current maximum penetration.
            self.maximum_penetration[index] = 0
        else:
            # Update the maximum penetration.
            self.maximum_penetration[index] = max(
                np.nan_to_num(self.touchdown_height[index] - msg.states[0].contact_positions[0].z),
                self.maximum_penetration[index])
            
    
    def get_maximum_penetration(self) -> np.ndarray:
        penetration =  np.maximum(
            np.nan_to_num(self.maximum_penetration),
            np.nan_to_num(self.maximum_penetration_previous_step)
        )
        
        return penetration
