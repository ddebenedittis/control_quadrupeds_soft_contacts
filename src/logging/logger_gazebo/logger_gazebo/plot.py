import csv
import os
import warnings

from cycler import cycler
# from scipy import signal
from matplotlib.cm import viridis
import matplotlib.pyplot as plt
import numpy as np
import quaternion

from robot_model.robot_model import RobotModel



def plot_colourline(x,y,c):
    col = viridis((c-np.min(c))/(np.max(c)-np.min(c)))
    ax = plt.gca()
    for i in np.arange(len(x)-1):
        ax.plot([x[i],x[i+1]], [y[i],y[i+1]], c=col[i])
    im = ax.scatter(x, y, c=c, s=0, cmap=viridis)
    return im

class Plot:
    """
    Save the csv data into numpy arrays and save several figures.
    """

    format = 'svg'

    def __init__(self, subdir = ""):
        [self.x_size_def, self.y_size_def] = plt.rcParams.get('figure.figsize')

        # Plot only data between time_init and time_end.
        self.time_init = 1
        self.time_end = 10

        self.foldername = 'log'

        self.subdir = subdir

        # =========================== Read The CSVs ========================== #

        params_file_path = self.foldername + '/csv/' + self.subdir + '/params.csv'
        with open(params_file_path, encoding='utf-8') as params_file:
            csv_reader = csv.reader(params_file, delimiter=',')
            for row in csv_reader:
                if row[0] == "robot_name":
                    self.robot_name = row[1]
                elif row[0] == "gait":
                    self.gait = row[1]
                elif row[0] == 'speed':
                    self.speed = float(row[1])

        fullfolder = self.foldername + '/csv/' + self.subdir + '/'

        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            
            def load_csv(fullfolder, filename):
                return np.loadtxt(fullfolder + filename, delimiter=",")

            self.contact_forces = load_csv(fullfolder, 'contact_forces.csv')
            self.contact_positions = load_csv(fullfolder, 'contact_positions.csv')
            self.depths = load_csv(fullfolder, 'depths.csv')
            self.feet_positions = load_csv(fullfolder, 'feet_positions.csv')
            self.feet_velocities = load_csv(fullfolder, 'feet_velocities.csv')
            self.generalized_coordinates = load_csv(fullfolder, 'generalized_coordinates.csv')
            self.generalized_velocities = load_csv(fullfolder, 'generalized_velocities.csv')
            self.optimal_deformations = load_csv(fullfolder, 'optimal_deformations.csv')
            self.optimal_forces = load_csv(fullfolder, 'optimal_forces.csv')
            self.optimal_joints_accelerations = load_csv(fullfolder, 'optimal_joints_accelerations.csv')
            self.optimal_torques = load_csv(fullfolder, 'optimal_torques.csv')
            self.orientation_error = load_csv(fullfolder, 'orientation_error.csv')
            self.position_error = load_csv(fullfolder, 'position_error.csv')
            if self.gait in ["walking_trot", "teleop_walking_trot"]:
                self.simple_velocity_command = load_csv(fullfolder, 'simple_velocity_command.csv')
            self.slippage = load_csv(fullfolder, 'slippage.csv')
            self.time = load_csv(fullfolder, 'time.csv')
            self.velocity_error = load_csv(fullfolder, 'velocity_error.csv')
            self.desired_feet_positions = load_csv(fullfolder, 'desired_feet_positions.csv')
            self.desired_feet_velocities = load_csv(fullfolder, 'desired_feet_velocities.csv')
            

        # Get the indexes of time_init and time_end in the time vector.
        for i, time_i in enumerate(self.time):
            if time_i >= self.time_init:
                self.i_init = i
                break

        for i in range(len(self.time)-1, -1, -1):
            if self.time[i] <= self.time_end:
                self.i_end = i
                break

        self.time_vector = self.time[self.i_init:self.i_end]

        self.feet_names = ["LF", "RF", "LH", "RH"]
        self.dir_names = ["x", "y", "z"]

        # ==================================================================== #

        # Compute the feet positions and velocities from the generalized coordinates and velocities vector.

        i_init = self.i_init
        i_end = self.i_end

        robot_model = RobotModel(self.robot_name)

        # The generalized accelerations vector is integrated to obtain the generalized coordinates and velocities vectors. The integration is performed over k_vec steps.
        k_vec = [1]

        q = self.generalized_coordinates
        u = self.generalized_velocities

        # Vectors that will contain the integrated generalized coordinates and velocities vectors.
        q_int = np.copy(q[i_init:i_end, :])
        u_int = np.copy(u[i_init:i_end, :])

        # Vectors that will contain the feet positions computed with q_int and u_int.
        self.feet_pos_int = np.zeros((i_end+1-i_init, 12))
        self.feet_vel_int = np.zeros((i_end+1-i_init, 12))

        # It may be interesting to perform this with different values of k.
        #! The code does not support yet the use of multiple values of k.
        for k in k_vec:
            self.feet_pos_meas = np.zeros((i_end+1+k-i_init, 12))
            self.feet_vel_meas = np.zeros((i_end+1+k-i_init, 12))

            for i in range(k):
                for j in range(i_init, i_end):
                    dt = self.time[j + 1 + i] - self.time[j + i]

                    w_old = u_int[j-i_init, 3:6]
                    w_new = w_old + self.optimal_joints_accelerations[j+i, 3:6] * dt
                    w_avg = 0.5 * (w_old + w_new)

                    q_int[j-i_init, 0:3] = \
                          q_int[j-i_init, 0:3] \
                        + u_int[j-i_init, 0:3] * dt \
                        + 0.5 * self.optimal_joints_accelerations[j+i, 0:3] * dt**2

                    quat = quaternion.from_float_array(q_int[j-i_init, 3:7])
                    quat = quat * (
                            quaternion.from_rotation_vector(w_avg * dt)
                        + dt**2/24 * quaternion.from_vector_part(np.cross(w_old, w_new))
                    )
                    quat = np.normalized(quat)
                    q_int[j-i_init, 3:7] = quaternion.as_float_array(quat)

                    q_int[j-i_init, 7:] = \
                          q_int[j-i_init, 7:] \
                        + u_int[j-i_init, 6:] * dt \
                        + 0.5 * self.optimal_joints_accelerations[j+i, 6:] * dt**2

                    u_int[j-i_init, :] = \
                        u_int[j-i_init, :] \
                        + self.optimal_joints_accelerations[j+i, :] * dt


            for j in range(i_init, i_end):
                [feet_pos, feet_vel] = robot_model.compute_feet_pos_vel(q_int[j-i_init,:], u_int[j-i_init,:])

                self.feet_pos_int[j-i_init,:] = feet_pos.flatten()
                self.feet_vel_int[j-i_init,:] = feet_vel.flatten()

            for j in range(i_init, i_end + k):
                [feet_pos, feet_vel] = robot_model.compute_feet_pos_vel(q[j,:], u[j,:])

                self.feet_pos_meas[j-i_init,:] = feet_pos.flatten()
                self.feet_vel_meas[j-i_init,:] = feet_vel.flatten()

    # ======================================================================== #
    #                              PLOT FUNCTIONS                              #
    # ======================================================================== #

    # Methods that plot different quantities. The method inputs are:
    # ax (pyplot.axes): the axes (or a list of axes) that will be populated
    # i (int): the index of the foot whose quantities are plotted:
    #          0 = LF, 1 = RF, 2 =  LH, 3 = RH
    
    def _plot_com_wrt_support(self, fig, ax: plt.Axes):
        
        support_polygon_center = np.nanmean(
            self.contact_positions[self.i_init:self.i_end,:].reshape((-1, 3, 4), order='F'),
            axis=2)
        
        im = plot_colourline(
            self.generalized_coordinates[self.i_init:self.i_end, 0] - support_polygon_center[:,0],
            self.generalized_coordinates[self.i_init:self.i_end, 1] - support_polygon_center[:,1],
            self.time_vector,
        )
        
        cbar = fig.colorbar(im)
        cbar.ax.set_title("Time [s]")

        ax.set(
            xlabel = "x-coordinate [m]",
            ylabel = "y-coordinate [m]",
            title = "CoM w.r.t. center of the support polygon",
        )
    

    def _plot_optimal_deformations(self, ax: plt.Axes, i):
        if self.optimal_deformations.size > 0:
            k = 1000

            def_size = int(self.optimal_deformations.shape[1] / 4)

            ax.plot(self.time_vector,
                    k * self.optimal_deformations[self.i_init:self.i_end,
                                                  def_size*i:def_size*(i+1)])

            ax.set(
                xlabel = "time [s]",
                ylabel = "deformations [mm]",
                title = "optimal deformations - " + self.feet_names[i],
            )
            if def_size == 3:
                ax.legend(["x-axis", "y-axis", "z-axis"], ncol=1)
            elif def_size == 1:
                ax.legend(["z-axis"])
            ax.set_xlim([self.time_init, self.time_end])
        else:
            ax.set(title="There's a time and place for everything, but not now.")


    def _plot_optimal_forces(self, ax: plt.Axes, i):
        ax.plot(self.time_vector,
                self.optimal_forces[self.i_init:self.i_end,
                                    3*i:3*i+3])

        ax.set(
            xlabel = "time [s]",
            ylabel = "forces [N]",
            title = "optimal forces - " + self.feet_names[i],
        )
        ax.legend(["x-axis", "y-axis", "z-axis"], ncol=1)
        ax.set_xlim([self.time_init, self.time_end])


    def _plot_optimal_torques(self, ax: plt.Axes, i):
        ax.plot(self.time_vector,
                self.optimal_torques[self.i_init:self.i_end,
                                     3*i:3*i+3])

        ax.set(
            xlabel = "time [s]",
            ylabel = "torques [Nm]",
            title = "optimal torques - " + self.feet_names[i],
        )
        ax.legend(["HAA", "HFE", "KFE"], ncol=3)
        ax.set_xlim([self.time_init, self.time_end])


    def _plot_tangential_over_normal_optimal_forces(self, ax):
        """
        Plot the ratio of the tangential and normal component of the contact forces of the four feet.
        """

        tangential_over_normal_optimal_force = np.zeros((self.i_end - self.i_init, 4))

        for i in range(4):
            # + 1e-9 in order to avoid dividing by zero.
            tangential_over_normal_optimal_force[:,i] = (
                  self.optimal_forces[self.i_init:self.i_end, 0+3*i]**2 \
                + self.optimal_forces[self.i_init:self.i_end, 1+3*i]**2
            )**0.5 / (self.optimal_forces[self.i_init:self.i_end, 2+3*i]**2 + 1e-9)


        ax.set(
            xlabel = "time [s]",
            ylabel = r"$f_t / f_n$",
            title = "ratio of the tangential and normal forces"
        )
        ax.legend(["LF", "RF", "LH", "RH"], ncol=2)
        ax.set_xlim([self.time_init, self.time_end])


    def _plot_meas_vs_opti_forces(self, axs, i):
        for j in range(3):
            axs[j].plot(self.time_vector,
                        self.contact_forces[self.i_init:self.i_end,
                                            j + 3*i])

            axs[j].plot(self.time_vector,
                        self.optimal_forces[self.i_init:self.i_end,
                                            j + 3*i])

            axs[j].set(
                xlabel = "time [s]",
                ylabel = "force [N]",
                title = "simulated vs optimized contact forces - " + self.dir_names[j] + "-axis" + " - " + self.feet_names[i]
            )
            axs[j].legend(["simulated forces", "optimized forces"])
            axs[j].set_xlim([self.time_init, self.time_end])

        axs[3].plot(self.time_vector,
                    (self.contact_forces[self.i_init:self.i_end, 0 + 3*i]**2 + self.contact_forces[self.i_init:self.i_end, 1 + 3*i]**2)**0.5)

        axs[3].plot(self.time_vector,
                    (self.optimal_forces[self.i_init:self.i_end, 0 + 3*i]**2 + self.optimal_forces[self.i_init:self.i_end, 1 + 3*i]**2)**0.5)

        axs[3].set(
            xlabel = "time [s]",
            ylabel = "force [N]",
            title = "simulated vs optimized contact forces - tangential - " + self.feet_names[i]
        )

        axs[3].legend(["simulated forces", "optimized forces"])
        axs[3].set_xlim([self.time_init, self.time_end])


    def _plot_meas_vs_opti_depths(self, ax: plt.Axes, i):
        if self.optimal_deformations.size > 0:
            k = int(self.optimal_deformations.shape[1] / 4)

            ax.plot(self.time_vector,
                    1000 * self.depths[self.i_init:self.i_end, i])

            ax.plot(self.time_vector,
                    1000 * self.optimal_deformations[self.i_init:self.i_end, k*i])

            ax.set(
                xlabel = "time [s]",
                ylabel = "depths [mm]",
                title = "measured vs optimized deformations - " + self.feet_names[i],
            )
            ax.legend(["measured", "optimized"])
        else:
            ax.set(title = "Not in my house!")


    def _plot_meas_vs_opti_foot_positions(self, axs, i):
        #! Considers that all feet are in contact with the terrain

        for j in range(3):
            axs[j].plot(self.time_vector,
                        self.feet_positions[self.i_init:self.i_end, j + 3*i])

            axs[j].plot(self.time_vector,
                        self.feet_pos_meas[0:self.i_end-self.i_init, j + 3*i])

            axs[j].set(
                xlabel = "time [s]",
                ylabel = "position [m]",
                title = "true vs measured foot position - " + self.dir_names[j] + "-axis" + " - " + self.feet_names[i]
            )
            axs[j].legend(["true", "measured"])
            axs[j].set_xlim([self.time_init, self.time_end])


    def _plot_meas_vs_opti_feet_velocities(self, axs, i):
        for j in range(3):
            axs[j].plot(self.time_vector,
                        self.feet_pos_int[0:self.i_end-self.i_init, j + 3*i] - self.feet_pos_meas[0:self.i_end-self.i_init, j + 3*i])

            #! +1 if previously k = [1]
            axs[j].plot(self.time_vector,
                        self.feet_pos_meas[1:self.i_end-self.i_init+1, j + 3*i] - self.feet_pos_meas[0:self.i_end-self.i_init, j + 3*i])

            axs[j].set(
                xlabel = "time [s]",
                ylabel = "temp [m]",
                title = "measured vs optimized foot velocity - " + self.dir_names[j] + "-axis" + " - " + self.feet_names[i]
            )
            axs[j].legend(["measured", "optimized"])
            axs[j].set_xlim([self.time_init, self.time_end])


    def _plot_meas_vs_opti_feet_accelerations(self, axs, i):
        for j in range(3):
            axs[j].plot(self.time_vector,
                        self.feet_vel_int[self.i_init:self.i_end, j + 3*i] - self.feet_pos_meas[0:self.i_end-self.i_init, j + 3*i])

            #! +1 if previously k = [1]
            axs[j].plot(self.time_vector,
                        self.feet_vel_meas[self.i_init+1:self.i_end+1, j + 3*i] - self.feet_pos_meas[0:self.i_end-self.i_init, j + 3*i])

            axs[j].set(
                xlabel = "time [s]",
                ylabel = "temp [m\s]",
                title = "measured vs optimized foot acceleration - " + self.dir_names[j] + "-axis" + " - " + self.feet_names[i]
            )
            axs[j].legend(["measured", "optimized"])
            axs[j].set_xlim([self.time_init, self.time_end])


    def _plot_slippage(self, ax):
        ax.plot(self.time_vector, self.slippage[self.i_init:self.i_end])

        ax.set(
            xlabel = "time [s]",
            ylabel = "slippage [m]",
            title = "slippage",
        )
        ax.set_xlim([self.time_init, self.time_end])


    def _plot_normalized_slippage(self, ax):
        if self.speed != 0:
            # + 1e-9 in order to avoid dividing by zero.
            ax.plot(self.time_vector,
                    self.slippage[self.i_init:self.i_end] / (abs(self.speed) * (self.time_vector) + 1e-9))

            ax.set(
                xlabel = "time [s]",
                ylabel = "normalized slippage",
                title = "normalized slippage",
            )
            ax.set_xlim([self.time_init, self.time_end])
        else:
            # This plot is pointless if the speed of the robot is zero.
            ax.set(title = "No, no, no.",)


    def _plot_reference_vs_measured_velocity(self, axs):
        dir_names = ["forward", "lateral"]
        
        quaternions = quaternion.from_float_array(
            self.generalized_coordinates[self.i_init:self.i_end, [6, 3, 4, 5]]
        )
        
        lin_vel_local_frame = np.zeros((self.i_end - self.i_init, 3))
        for i in range(self.i_end - self.i_init):
            lin_vel_local_frame[i,:] = quaternion.as_vector_part(
                quaternions[i].conjugate() * quaternion.from_vector_part(
                    self.generalized_velocities[self.i_init + i, 0:3]
                ) * quaternions[i]
            )

        for i in range(3):
            axs[i].plot(self.time_vector,
                        self.simple_velocity_command[self.i_init:self.i_end, i])

            if i != 2:
                axs[i].plot(self.time_vector,
                            lin_vel_local_frame[:,i])
                axs[i].set(
                    xlabel = "time [s]",
                    ylabel = "velocity [m/s]",
                    title = "commanded vs measured " + dir_names[i] + " velocity",
                )
            elif i == 2:
                axs[i].plot(self.time_vector,
                            self.generalized_velocities[self.i_init:self.i_end, 5])

                axs[i].set(
                    xlabel = "time [s]",
                    ylabel = "angular velocity [rad/s]",
                    title = "commanded vs measured yaw rate"
                )


            axs[i].legend(["commanded", "measured"])
            axs[i].set_xlim([self.time_init, self.time_end])


    def _plot_desired_vs_true_foot_pos(self, axs, foot_idx: int):
        for i in range(3):
            axs[i].plot(self.time_vector,
                        self.feet_positions[self.i_init:self.i_end, 3*foot_idx + i],
                        marker=".")
            
            axs[i].plot(self.time_vector,
                        self.desired_feet_positions[self.i_init:self.i_end, 3*foot_idx + i],
                        marker=".")
            
            axs[i].set(
                xlabel = "time [s]",
                ylabel = "position [m]",
                title = "measured vs desired " + self.feet_names[foot_idx] + " foot position" \
                    + " - " + self.dir_names[i] + " coordinate",
            )
            
            axs[i].legend(["measured", "reference"])
            axs[i].set_xlim([self.time_init, self.time_end])


    def _plot_feet_pos_error(self, ax, foot_idx: int):
        ax.plot(self.time_vector,
                (self.feet_positions[self.i_init+1:self.i_end+1, 3*foot_idx:3*foot_idx+3]
                - self.desired_feet_positions[self.i_init:self.i_end, 3*foot_idx:3*foot_idx+3]),
                marker=".")

        ax.set(
            xlabel = "time [s]",
            ylabel = "position [m]",
            title = self.feet_names[foot_idx] + " foot position error"
        )

        ax.legend(["x-coordinate", "y-coordinate", "z-coordinate"])

        ax.set_xlim([self.time_init, self.time_end])
        
    
    def _plot_desired_vs_true_foot_vel(self, axs, foot_idx: int):
        for i in range(3):
            axs[i].plot(self.time_vector,
                        self.feet_velocities[self.i_init:self.i_end, 3*foot_idx + i],
                        marker=".")
            
            axs[i].plot(self.time_vector,
                        self.desired_feet_velocities[self.i_init:self.i_end, 3*foot_idx + i],
                        marker=".")
            
            axs[i].set(
                xlabel = "time [s]",
                ylabel = "velocity [m/s]",
                title = "measured vs desired " + self.feet_names[foot_idx] + " foot velocity" \
                    + " - " + self.dir_names[i] + " coordinate",
            )
            
            axs[i].legend(["measured", "reference"])
            axs[i].set_xlim([self.time_init, self.time_end])


    def _plot_feet_vel_error(self, ax, foot_idx: int):
        ax.plot(self.time_vector,
                (self.feet_velocities[self.i_init+1:self.i_end+1, 3*foot_idx:3*foot_idx+3]
                - self.desired_feet_velocities[self.i_init:self.i_end, 3*foot_idx:3*foot_idx+3]),
                marker=".")

        ax.set(
            xlabel = "time [s]",
            ylabel = "velocity [m/s]",
            title = self.feet_names[foot_idx] + " foot velocity error"
        )

        ax.legend(["x-coordinate", "y-coordinate", "z-coordinate"])

        ax.set_xlim([self.time_init, self.time_end])


    def _save_megaplot_i(self, i):
        fig, axs = plt.subplots(4, 4, figsize=[3.5*self.x_size_def, 2.5*self.y_size_def], layout="constrained")

        self._plot_meas_vs_opti_forces(axs[0:4, 0], i)
        self._plot_meas_vs_opti_depths(axs[0,1], i)
        self._plot_optimal_deformations(axs[1,1], i)
        self._plot_optimal_forces(axs[2,1], i)
        self._plot_optimal_torques(axs[3,1], i)
        self._plot_meas_vs_opti_foot_positions(axs[0:3,2], i)
        self._plot_meas_vs_opti_feet_velocities(axs[0:3,3], i)

        fig_path = os.path.join(self.foldername, self.format, self.subdir,
                                "megaplot_" + self.feet_names[i] + '.' + self.format)
        plt.savefig(fig_path, bbox_inches="tight", format=self.format)
        plt.close(fig)
        

    def _save_meas_vs_opti_forces_plot(self):
        fig, axs = plt.subplots(4, 4, figsize=[3.5*self.x_size_def, 2.5*self.y_size_def])
        for i in range(len(self.feet_names)):
            self._plot_meas_vs_opti_forces(axs[0:4, i], i)

        fig_path = os.path.join(self.foldername, self.format, self.subdir,
                                'meas_vs_opti_forces.' + self.format)
        plt.savefig(fig_path, bbox_inches="tight", format=self.format)
        plt.close(fig)


    def _save_performance_plots(self):
        fig, axs = plt.subplots(3, 2, figsize=[self.x_size_def, 2*self.y_size_def])
        self._plot_slippage(axs[0,0])
        self._plot_normalized_slippage(axs[1,0])

        if self.gait in ["walking_trot", "teleop_walking_trot"]:
            self._plot_reference_vs_measured_velocity(axs[0:3,1])

        fig_path = os.path.join(self.foldername, self.format, self.subdir,
                                'performance.' + self.format)
        plt.savefig(fig_path, bbox_inches="tight", format=self.format)
        plt.close(fig)


    def _save_feet_pos_plots(self):
        fig, axs = plt.subplots(2, 4, figsize=[3.5*self.x_size_def, 1.5*self.y_size_def])
        
        foot_idx = 0
        
        self._plot_desired_vs_true_foot_pos(axs[0,0:3], foot_idx = foot_idx)
        self._plot_feet_pos_error(axs[0,3], foot_idx = foot_idx)
        
        self._plot_desired_vs_true_foot_vel(axs[1,0:3], foot_idx = foot_idx)
        self._plot_feet_vel_error(axs[1,3], foot_idx = foot_idx)
        
        fig_path = os.path.join(self.foldername, self.format, self.subdir,
                                'feet_pos_vel.' + self.format)
        plt.savefig(fig_path, bbox_inches="tight", format=self.format)
        plt.close(fig)
        
        
    def _save_performance_indexes(self):
        quaternions = quaternion.from_float_array(
            self.generalized_coordinates[self.i_init:self.i_end, [6, 3, 4, 5]]
        )
        
        lin_vel_local_frame = np.zeros((self.i_end - self.i_init, 3))
        for i in range(self.i_end - self.i_init):
            lin_vel_local_frame[i,:] = quaternion.as_vector_part(
                quaternions[i].conjugate() * quaternion.from_vector_part(
                    self.generalized_velocities[self.i_init + i, 0:3]
                ) * quaternions[i]
            )
            
        # sos = signal.butter(4, 20, 'hp', fs=100, output='sos')
        # lin_vel_local_frame[:,0] = signal.sosfilt(sos, lin_vel_local_frame[:,0])
        # lin_vel_local_frame[:,1] = signal.sosfilt(sos, lin_vel_local_frame[:,1])
            
        rmse_heading = (((
            lin_vel_local_frame[:,0] - self.simple_velocity_command[self.i_init:self.i_end, 0]
        )**2).mean())**0.5
        rmse_lateral = (((
            lin_vel_local_frame[:,1] - self.simple_velocity_command[self.i_init:self.i_end, 1]
        )**2).mean())**0.5
        rmse_yaw = (((
            self.generalized_velocities[self.i_init:self.i_end, 5] - self.simple_velocity_command[self.i_init:self.i_end, 2]
        )**2).mean())**0.5
        
        vel_cmd = self.simple_velocity_command[self.i_init:self.i_end,0]
        transport = (
              lin_vel_local_frame[:,0] * vel_cmd / np.abs(vel_cmd) * (self.time_vector[1] - self.time_vector[0])
        ).sum()
        # transport = np.abs(
        #       lin_vel_local_frame[:,0] * vel_cmd / np.abs(vel_cmd) * (self.time_vector[1] - self.time_vector[0])
        # ).sum()
        torques_cost = (
            ( self.optimal_torques[self.i_init:self.i_end, 0]**2
            + self.optimal_torques[self.i_init:self.i_end, 1]**2
            + self.optimal_torques[self.i_init:self.i_end, 2]**2) 
        * (self.time_vector[1] - self.time_vector[0])).sum()
        
        kpi_dict = {
            'rmse_heading': rmse_heading,
            'rmse_lateral': rmse_lateral,
            'rmse_yaw': rmse_yaw,
            'slippage': self.slippage[self.i_end],
            'CoT': transport/torques_cost,
            'torques_cost': torques_cost,
        }
        
        np.save(self.foldername + '/csv/' + self.subdir + '/kpi.npy', kpi_dict)
        
        
    def _save_com_wrt_support(self):
        fig, ax = plt.subplots(1, 1, figsize=[self.x_size_def, self.y_size_def])
        
        self._plot_com_wrt_support(fig, ax)
        
        fig_path = os.path.join(self.foldername, self.format, self.subdir,
                                'com_wrt_support.' + self.format)
        plt.savefig(fig_path, bbox_inches="tight", format=self.format)
        plt.close(fig)


    def save_all_plots(self):
        """
        Save all the plots.
        """

        for i in range(len(self.feet_names)):
            self._save_megaplot_i(i)

        self._save_meas_vs_opti_forces_plot()
        self._save_performance_plots()
        self._save_feet_pos_plots()
        
        self._save_performance_indexes()
        
        self._save_com_wrt_support()



def main():
    # ======================== Plots Style Definition ======================== #

    default_cycler = (
        cycler(color=['#0072BD', '#D95319', '#EDB120', '#7E2F8E']) +
        cycler('linestyle', ['-', '--', '-', '--'])
    )

    textsize = 12
    labelsize = 16

    plt.rc('font', family='serif', serif='Times')
    plt.rc('text', usetex=True)
    plt.rc('xtick', labelsize=textsize)
    plt.rc('ytick', labelsize=textsize)
    plt.rc('axes', labelsize=labelsize, prop_cycle=default_cycler)
    plt.rc('legend', fontsize=textsize)

    plt.rc("axes", grid=True)
    plt.rc("grid", linestyle='dotted', linewidth=0.25)

    plt.rcParams['figure.constrained_layout.use'] = True

    plots_format = Plot.format

    rootdir_csv = "log/csv"
    rootdir_plt = "log/" + plots_format
    n_to_process = 0
    for file in os.listdir(rootdir_csv):
        d = os.path.join(rootdir_csv, file)
        if os.path.isdir(d):
            os.makedirs(os.path.join(rootdir_plt, file), exist_ok=True)
            if len(os.listdir(os.path.join(rootdir_plt, file))) == 0:
                n_to_process += 1


    counter = 0
    for file in os.listdir(rootdir_csv):
        d = os.path.join(rootdir_csv, file)
        if os.path.isdir(d) \
            and len(os.listdir(os.path.join(rootdir_plt, file))) == 0:

            counter += 1
            print("Processing the " + \
                  str(counter) + \
                  "-th folder out of " + \
                  str(n_to_process) + \
                  " total folders (" +
                  file +
                  ")...")

            plot = Plot(file)
            plot.save_all_plots()

    print("\nFinished.\n")



if __name__ == '__main__':
    main()
