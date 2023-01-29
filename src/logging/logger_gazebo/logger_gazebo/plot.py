import os
import shutil

from cycler import cycler
import matplotlib.pyplot as plt
import numpy as np
import quaternion

from robot_model.robot_model import RobotModel



def main():
    
    # ============================= Preliminaries ============================ #
    
    default_cycler = (cycler(color=['#0072BD', '#D95319', '#EDB120', '#7E2F8E']) + 
                    cycler('linestyle', ['-', '--', '-', '--']))

    textsize = 16

    plt.rc('font', family='serif', serif='Times')
    plt.rc('text', usetex=True)
    plt.rc('xtick', labelsize=textsize)
    plt.rc('ytick', labelsize=textsize)
    plt.rc('axes', labelsize=24, prop_cycle=default_cycler)
    plt.rc('legend', fontsize=textsize)
    
    plt.rc("axes", grid=True)
    plt.rc("grid", linestyle='dotted', linewidth=0.25)
    
    
    # Get the default figure size
    [x_size_def, y_size_def] = plt.rcParams.get('figure.figsize')

    # Figures output format
    format = 'svg'
    
    # x-axis (time) limits
    time_init = 0
    time_end = 10

    # Names of the csv files
    filenames = [
        'optimal_deformations',
        'optimal_forces',
        'optimal_torques',
        'orientation_error',
        'position_error',
        'slippage',
        'velocity_error',
        'contact_forces',
        'optimal_joints_accelerations',
    ]

    # The same operations must be repeated for the csv files in all these folders
    foldernames = [
        'log'
    ]
    
    
    # ======================================================================== #
    
    for foldername in foldernames:
        
        # Remove these folders and their content
        shutil.rmtree(foldername + "/pdf", ignore_errors=True)
        shutil.rmtree(foldername + "/svg", ignore_errors=True)
        
        # Create a new folder
        os.makedirs(foldername + "/" + format)

        time_vector = np.loadtxt(foldername + '/csv/' + 'time' + '.csv', delimiter=",")

        # Get the indexes of t_init and t_end in time_vector
        for i in range(len(time_vector)):
            if time_vector[i] >= time_init:
                i_init = i
                break
            
        for i in range(len(time_vector)-1, -1, -1):
            if time_vector[i] <= time_end:
                i_end = i
                break


        for filename in filenames:
            
            # ================================================================ #
            
            # Plot the integrated optimal joints accelerations
            
            if filename == 'optimal_joints_accelerations':
                
                robot_name = "anymal_c"
                robot_model = RobotModel(robot_name)
            
                u_dot_star = np.loadtxt(foldername + '/csv/' + filename + '.csv', delimiter=",")
                u = np.loadtxt(foldername + '/csv/' + 'generalized_velocities' + '.csv', delimiter=",")
                q = np.loadtxt(foldername + '/csv/' + 'generalized_coordinates' + '.csv', delimiter=",")
                
                feet_pos_meas = np.loadtxt(foldername + '/csv/' + 'feet_positions' + '.csv', delimiter=",")
                feet_vel_meas = np.loadtxt(foldername + '/csv/' + 'feet_velocities' + '.csv', delimiter=",")
                
                k_vec = [1]
                
                u_int = np.copy(u[i_init:i_end, :])
                q_int = np.copy(q[i_init:i_end, :])
                
                feet_pos_int = np.zeros((i_end+1-i_init, 12))
                feet_vel_int = np.zeros((i_end+1-i_init, 12))
                
                for k in k_vec:
                    feet_pos_meas2 = np.zeros((i_end+1+k-i_init, 12))
                    feet_vel_meas2 = np.zeros((i_end+1+k-i_init, 12))
                    
                    for i in range(k):
                        for j in range(i_init, i_end):
                            dt = time_vector[j + 1 + i] - time_vector[j + i]
                        
                            w_old = u_int[j, 3:6]
                            w_new = w_old + u_dot_star[j+i, 3:6] * dt
                            w_avg = 0.5 * (w_old + w_new)

                            q_int[j, 0:3] = q_int[j, 0:3] + u_int[j, 0:3] * dt + 0.5 * u_dot_star[j+i, 0:3] * dt**2
                            
                            quat = quaternion.from_float_array(q_int[j, 3:7])
                            quat = quat * (
                                  quaternion.from_rotation_vector(w_avg * dt)
                                + dt**2/24 * quaternion.from_vector_part(np.cross(w_old, w_new))
                            )
                            quat = np.normalized(quat)
                            q_int[j, 3:7] = quaternion.as_float_array(quat)
                            
                            q_int[j, 7:] = q_int[j, 7:] + u_int[j, 6:] * dt + 0.5 * u_dot_star[j+i, 6:] * dt**2
                            
                            u_int[j, :] = u_int[j, :] + u_dot_star[j+i, :] * dt

                                                    
                    for j in range(i_init, i_end):
                        [feet_pos, feet_vel] = robot_model.compute_feet_pos_vel(q_int[j,:], u_int[j,:])
                        
                        feet_pos_int[j,:] = feet_pos.flatten()
                        feet_vel_int[j,:] = feet_vel.flatten()
                        
                    for j in range(i_init, i_end + k):
                        [feet_pos, feet_vel] = robot_model.compute_feet_pos_vel(q[j,:], u[j,:])
                        
                        feet_pos_meas2[j,:] = feet_pos.flatten()
                        feet_vel_meas2[j,:] = feet_vel.flatten()
                        
                    
                    arr1 = feet_pos_int[i_init:i_end, :] - feet_pos_meas2[i_init:i_end, :]
                    arr2 = feet_pos_meas2[i_init+k:i_end+k, :] - feet_pos_meas2[i_init:i_end, :]
                    
                    arr3 = feet_vel_int[i_init:i_end, :] - feet_vel_meas2[i_init:i_end, :]
                    arr4 = feet_vel_meas2[i_init+k:i_end+k, :] - feet_vel_meas2[i_init:i_end, :]
                    
                    
                    parts = ['_LF', '_RF', '_LH', '_RH']
                    parts_indexes = [
                        0, 3,
                        3, 6,
                        6, 9,
                        9,12,
                    ]
                    
                    # Create one subplot graph for each foot (LF, RF, ...)
                    for i in range(len(parts)):
                        fig, axs = plt.subplots(3, figsize=[x_size_def, 2*y_size_def])
                        for j in range(3):
                            axs[j].plot(time_vector[i_init:i_end], arr1[i_init:i_end, j + parts_indexes[2*i]])
                            axs[j].plot(time_vector[i_init:i_end], arr2[i_init:i_end, j + parts_indexes[2*i]])
                            
                            dir_names = ["x", "y", "z"]
                            axs[j].set(
                                xlabel = "time [s]",
                                ylabel = "position [m]",
                                title = dir_names[j] + "-axis"
                            )
                            
                            axs[j].legend(["integrated position change", "measured position change"])
                            
                            axs[j].set_xlim([time_init, time_end])
                            
                        plt.savefig(foldername + '/' + format + '/' + 'integrated_position_change' + parts[i] + '.' + format, bbox_inches="tight", format=format)
                        plt.close(fig)
                        
                    for i in range(len(parts)):
                        fig, axs = plt.subplots(3, figsize=[x_size_def, 2*y_size_def])
                        for j in range(3):
                            axs[j].plot(time_vector[i_init:i_end], arr3[i_init:i_end, j + parts_indexes[2*i]])
                            axs[j].plot(time_vector[i_init:i_end], arr4[i_init:i_end, j + parts_indexes[2*i]])
                            
                            dir_names = ["x", "y", "z"]
                            axs[j].set(
                                xlabel = "time [s]",
                                ylabel = "velocity [m/s]",
                                title = dir_names[j] + "-axis"
                            )
                            
                            axs[j].legend(["integrated velocity change", "measured velocity change"])
                            
                            axs[j].set_xlim([time_init, time_end])
                            
                        plt.savefig(foldername + '/' + format + '/' + 'integrated_velocity_change' + parts[i] + '.' + format, bbox_inches="tight", format=format)
                        plt.close(fig)
                    
                    
                continue
                
            
            # ================================================================ #
            
            # Plot the optimized forces vs the measured forces
            
            if filename == "contact_forces":
                contact_forces = np.loadtxt(foldername + '/csv/' + filename + '.csv', delimiter=",")
                optimal_forces = np.loadtxt(foldername + '/csv/' + "optimal_forces" + '.csv', delimiter=",")
                
                parts = ['_LF', '_RF', '_LH', '_RH']
                parts_indexes = [
                    0, 3,
                    3, 6,
                    6, 9,
                    9,12,
                ]
                
                # Create one subplot graph for each foot (LF, RF, ...)
                for i in range(len(parts)):
                    fig, axs = plt.subplots(4, figsize=[x_size_def, 2*y_size_def])
                    for j in range(3):
                        axs[j].plot(time_vector[i_init:i_end], contact_forces[i_init:i_end, j + parts_indexes[2*i]])
                        axs[j].plot(time_vector[i_init:i_end], optimal_forces[i_init:i_end, j + parts_indexes[2*i]])
                        
                        dir_names = ["x", "y", "z"]
                        axs[j].set(
                            xlabel = "time [s]",
                            ylabel = "force [N]",
                            title = dir_names[j] + "-axis"
                        )
                        
                        axs[j].legend(["simulated forces", "optimized forces"])
                        
                        axs[j].set_xlim([time_init, time_end])
                        # axs[j].grid(linestyle='dotted', linewidth = 0.25)
                        
                    axs[3].plot(time_vector[i_init:i_end], (contact_forces[i_init:i_end, 0 + parts_indexes[2*i]]**2 + contact_forces[i_init:i_end, 1 + parts_indexes[2*i]]**2)**0.5)
                    axs[3].plot(time_vector[i_init:i_end], (optimal_forces[i_init:i_end, 0 + parts_indexes[2*i]]**2 + optimal_forces[i_init:i_end, 1 + parts_indexes[2*i]]**2)**0.5)
                    
                    axs[3].set(
                        xlabel = "time [s]",
                        ylabel = "force [N]",
                        title = "tangential"
                    )
                    
                    axs[3].legend(["simulated forces", "optimized forces"])
                    axs[3].set_xlim([time_init, time_end])
                    # axs[3].grid(linestyle='dotted', linewidth = 0.25)
                        
                    plt.savefig(foldername + '/' + format + '/' + filename + parts[i] + '.' + format, bbox_inches="tight", format=format)
                    plt.close(fig)
                    
                continue 
            
            arr = np.loadtxt(foldername + '/csv/' + filename + '.csv', delimiter=",")
            
            if filename == 'optimal_deformations':
                k = 1000
            elif filename == 'orientation_error':
                k = 180 / 2 / np.pi
            elif filename == 'position_error':
                k = 100
            else:
                k=1
                
            if filename == 'optimal_deformations':
                if arr.size == 0:
                    continue
                else:
                    arr2 = np.loadtxt(foldername + '/csv/' + 'depths' + '.csv', delimiter=",")
                    
                    parts = ["LF", "RF", "LH", "RH"]
                    
                    for i in range(4):
                        fig = plt.figure()
                        
                        if arr[0,:].size == 4:
                            plt.plot(time_vector[i_init:i_end], k *  arr[i_init:i_end, i])
                        else:
                            plt.plot(time_vector[i_init:i_end], k *  arr[i_init:i_end, 2+3*i])
                        plt.plot(time_vector[i_init:i_end], k * arr2[i_init:i_end, i])
                        
                        plt.ylabel("deformation [mm]")
                        plt.xlabel("time [s]")
                        plt.legend(["desired", "measured"])
                        
                        plt.xlim([time_init, time_end])
                        # plt.grid(linestyle='dotted', linewidth = 0.25)
                        plt.savefig(foldername + '/' + format + '/' + "deformations_" + parts[i] + '.' + format, bbox_inches="tight", format=format)
                        plt.close(fig)
                        
                    if arr[0,:].size == 4:
                        continue
                    
                
            if filename in ['optimal_deformations', 'optimal_forces', 'optimal_torques']:
                parts = ['_LF', '_RF', '_LH', '_RH']
                parts_indexes = [0, 3,
                                 3, 6,
                                 6, 9,
                                 9,12]
            else:
                parts = ['']
                
            for i in range(len(parts)):
                
                fig = plt.figure()
                if len(parts) > 1:
                    plt.plot(time_vector[i_init:i_end], k * arr[i_init:i_end, parts_indexes[2*i]:parts_indexes[2*i+1]])
                else:
                    plt.plot(time_vector[i_init:i_end], k * arr[i_init:i_end])
                
                if filename == 'optimal_deformations':
                    plt.ylabel("deformation [mm]")
                    plt.xlabel("time [s]")
                    plt.legend(["x-axis", "y-axis", "z-axis"], ncol=1, loc='upper right')
                elif filename == 'optimal_forces':
                    plt.ylabel("force [N]")
                    plt.xlabel("time [s]")
                    plt.legend(["x-axis", "y-axis", "z-axis"], ncol=3)
                elif filename == 'optimal_torques':
                    plt.ylabel("torque [Nm]")
                    plt.xlabel("time [s]")
                    plt.legend(["HAA", "HFE", "KFE"], ncol=3)
                elif filename == 'orientation_error':
                    plt.ylabel("orientation [deg]")
                    plt.xlabel("time [s]")
                elif filename == 'position_error':                
                    plt.ylabel("position [cm]")
                    plt.xlabel("time [s]")
                    plt.legend(["x-axis", "y-axis", "z-axis"], ncol=3, loc='lower right')
                elif filename == 'slippage':
                    plt.ylabel("meters [m]")
                    plt.xlabel("time [s]")
                elif filename == 'velocity_error':
                    plt.ylabel("velocity [m/s]")
                    plt.xlabel("time [s]")
                    plt.legend(["x-axis", "y-axis", "z-axis"], ncol=3)
                    
                plt.xlim([time_init, time_end])

                # plt.grid(linestyle='dotted', linewidth = 0.25)
                    
                plt.savefig(foldername + '/' + format + '/' + filename + parts[i] + '.' + format, bbox_inches="tight", format=format)

                plt.close(fig)
                
            
            # ============= Tangential Over Normal Contact Force ============= #
            
            if filename == 'optimal_forces':
                tangential_over_normal_optimal_force = np.zeros((i_end - i_init, 4))
                
                arr2  = arr[i_init:i_end,:]
                
                for i in range(4):
                    tangential_over_normal_optimal_force[:,i] = (arr2[:,0+3*i]**2 + arr2[:,1+3*i]**2)**0.5 / (arr2[:,2+3*i] + 1e-9)
                    
                fig = plt.figure()
                                    
                plt.plot(time_vector[i_init:i_end], tangential_over_normal_optimal_force)
                
                plt.ylabel(r"$f_{t} / f_{n}$")
                plt.xlabel("time [s]")
                plt.legend(["LF", "RF", "LH", "RH"], ncol=2)
                
                plt.xlim([time_init, time_end])
                # plt.grid(linestyle='dotted', linewidth = 0.25)
                plt.savefig(foldername + '/' + format + '/' + 'tangential_over_normal_optimal_force' + '.' + format, bbox_inches="tight", format=format)
                plt.close(fig)
                
                
                
if __name__ == '__main__':
    main()