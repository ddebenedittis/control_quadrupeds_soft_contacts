from ament_index_python.packages import get_package_share_directory
import numpy as np
import pinocchio as pin
import yaml



class RobotModel:
    
    def __init__(self, robot_name):
        
        # ======================== Read The YAML File ======================== #
        
        package_share_directory = get_package_share_directory('robot_model')
        
        yaml_path = package_share_directory + "/robots/all_robots.yaml"
        
        with open(yaml_path, 'r') as f:
            doc = yaml.load(f, yaml.SafeLoader)
            
        pkg_name = doc[robot_name]["pkg_name"]
        urdf_path = doc[robot_name]["urdf_path"]
        
        full_urdf_path = get_package_share_directory(pkg_name) + urdf_path
        
        self.feet_names = doc[robot_name]["feet_names"]
        self.ankle_feet_displacement = doc[robot_name]["ankle_feet_displacement"]
        
        
        # ============ Create The Robot Model And The Data Objects =========== #
        
        self._model = pin.buildModelFromUrdf(full_urdf_path, pin.JointModelFreeFlyer())
        
        self._data = self._model.createData()
        
        
    def compute_feet_pos_vel(self, qj, qj_dot, contact_feet_names):
        """
        Compute:
            B_r_fb : position of the feet with respect to the robot base, in base frame.
            B_v_fb : velocity of the feet with respect to the velocity of the robot base, in base frame.
        """
        
        q = np.concatenate((np.zeros(6), np.array([1]), qj))
        u = np.concatenate((np.zeros(6), qj_dot))
        
        pin.forwardKinematics(self._model, self._data, q, u)
        
        B_r_fb = np.zeros((len(contact_feet_names), 3))
        B_v_fb = np.zeros((len(contact_feet_names), 3))
        
        for i in range(len(contact_feet_names)):
            frame_id = self._model.getFrameId(contact_feet_names[i])
            
            pin.updateFramePlacement(self._model, self._data, frame_id)
            
            B_r_fb[i, :] = self._data.oMf[frame_id].translation
            B_v_fb[i, :] = pin.getFrameVelocity(self._model, self._data, frame_id, pin.LOCAL_WORLD_ALIGNED).linear
            
        return [B_r_fb, B_v_fb]