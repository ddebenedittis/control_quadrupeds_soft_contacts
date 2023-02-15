import csv
import os
import shutil

from cycler import cycler
import matplotlib.pyplot as plt
import numpy as np
import quaternion

from robot_model.robot_model import RobotModel



class Plot:
    """
    Save the csv data into numpy arrays and plot several figures.
    """
    
    def __init__(self):
        [self.x_size_def, self.y_size_def] = plt.rcParams.get('figure.figsize')
        
        self.format = 'svg'
        
        # Plot only data between time_init and time_end.
        self.time_init = 0
        self.time_end = 10
        
        self.foldername = 'log'
        
        # =========================== Read The CSVs ========================== #
        
        self.contact_forces = np.loadtxt(self.foldername + '/csv/' + 'contact_forces' + '.csv', delimiter=",")
        self.contact_positions = np.loadtxt(self.foldername + '/csv/' + 'contact_positions' + '.csv', delimiter=",")
        self.depths = np.loadtxt(self.foldername + '/csv/' + 'depths' + '.csv', delimiter=",")
        self.feet_positions = np.loadtxt(self.foldername + '/csv/' + 'feet_positions' + '.csv', delimiter=",")
        self.feet_velocities = np.loadtxt(self.foldername + '/csv/' + 'feet_velocities' + '.csv', delimiter=",")
        self.generalized_coordinates = np.loadtxt(self.foldername + '/csv/' + 'generalized_coordinates' + '.csv', delimiter=",")
        self.generalized_velocities = np.loadtxt(self.foldername + '/csv/' + 'generalized_velocities' + '.csv', delimiter=",")
        self.optimal_deformations = np.loadtxt(self.foldername + '/csv/' + 'optimal_deformations' + '.csv', delimiter=",")
        self.optimal_forces = np.loadtxt(self.foldername + '/csv/' + 'optimal_forces' + '.csv', delimiter=",")
        self.optimal_joints_accelerations = np.loadtxt(self.foldername + '/csv/' + 'optimal_joints_accelerations' + '.csv', delimiter=",")
        self.optimal_torques = np.loadtxt(self.foldername + '/csv/' + 'optimal_torques' + '.csv', delimiter=",")
        self.orientation_error = np.loadtxt(self.foldername + '/csv/' + 'orientation_error' + '.csv', delimiter=",")
        self.position_error = np.loadtxt(self.foldername + '/csv/' + 'position_error' + '.csv', delimiter=",")
        self.slippage = np.loadtxt(self.foldername + '/csv/' + 'slippage' + '.csv', delimiter=",")
        self.time = np.loadtxt(self.foldername + '/csv/' + 'time' + '.csv', delimiter=",")
        self.velocity_error = np.loadtxt(self.foldername + '/csv/' + 'velocity_error' + '.csv', delimiter=",")
        
        with open(self.foldername + '/csv/params.csv') as csv_file:
            csv_reader = csv.reader(csv_file, delimiter=',')
            for row in csv_reader:
                if row[0] == "robot_name":
                    self.robot_name = row[1]
                elif row[0] == 'speed':
                    self.speed = float(row[1])
        
        # Get the indexes of time_init and time_end in the time vector.
        for i in range(len(self.time)):
            if self.time[i] >= self.time_init:
                self.i_init = i
                break
            
        for i in range(len(self.time)-1, -1, -1):
            if self.time[i] <= self.time_end:
                self.i_end = i
                break
            
        self.time_vector = self.time[self.i_init:self.i_end]
        
        self.foot_names = ["LF", "RF", "LH", "RH"]
        self.dir_names = ["x", "y", "z"]
        
        # ==================================================================== #
        
        # Compute the feet positions and velocities from the generalized coordinates and velocities vector.
        
        i_init = self.i_init
        i_end = self.i_end
        
        robot_name = "anymal_c"
        robot_model = RobotModel(robot_name)
        
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
                
                    w_old = u_int[j, 3:6]
                    w_new = w_old + self.optimal_joints_accelerations[j+i, 3:6] * dt
                    w_avg = 0.5 * (w_old + w_new)

                    q_int[j, 0:3] = q_int[j, 0:3] + u_int[j, 0:3] * dt + 0.5 * self.optimal_joints_accelerations[j+i, 0:3] * dt**2
                    
                    quat = quaternion.from_float_array(q_int[j, 3:7])
                    quat = quat * (
                            quaternion.from_rotation_vector(w_avg * dt)
                        + dt**2/24 * quaternion.from_vector_part(np.cross(w_old, w_new))
                    )
                    quat = np.normalized(quat)
                    q_int[j, 3:7] = quaternion.as_float_array(quat)
                    
                    q_int[j, 7:] = q_int[j, 7:] + u_int[j, 6:] * dt + 0.5 * self.optimal_joints_accelerations[j+i, 6:] * dt**2
                    
                    u_int[j, :] = u_int[j, :] + self.optimal_joints_accelerations[j+i, :] * dt

                                            
            for j in range(i_init, i_end):
                [feet_pos, feet_vel] = robot_model.compute_feet_pos_vel(q_int[j,:], u_int[j,:])
                
                self.feet_pos_int[j,:] = feet_pos.flatten()
                self.feet_vel_int[j,:] = feet_vel.flatten()
                
            for j in range(i_init, i_end + k):
                [feet_pos, feet_vel] = robot_model.compute_feet_pos_vel(q[j,:], u[j,:])
                
                self.feet_pos_meas[j,:] = feet_pos.flatten()
                self.feet_vel_meas[j,:] = feet_vel.flatten()
        
    # ======================================================================== #
    #                              PLOT FUNCTIONS                              #
    # ======================================================================== #
    
    # Methods that plot different quantities. The method inputs are:
    # ax (pyplot.axes): the axes (or a list of axes of appropriate dimension) that will be populated
    # i (int): the index of the foot whose quantities are plotted. 0 = LF, 1 = RF, 2 =  LH, 3 = RH
        
    def plot_optimal_deformations(self, ax, i):
        if self.optimal_deformations.size > 0:
            k = 1000
            
            def_size = int(self.optimal_deformations.shape[1] / 4)
            
            ax.plot(self.time_vector,
                    k * self.optimal_deformations[self.i_init:self.i_end,
                                                  def_size*i:def_size*(i+1)])
            
            ax.set(
                xlabel = "time [s]",
                ylabel = "deformations [mm]",
                title = "optimal deformations - " + self.foot_names[i],
            )
            if def_size == 3:
                ax.legend(["x-axis", "y-axis", "z-axis"], ncol=1)
            elif def_size == 1:
                ax.legend(["z-axis"])
            ax.set_xlim([self.time_init, self.time_end])
        else:
            ax.set(title="There's a time and place for everything, but not now.")
        
    
    def plot_optimal_forces(self, ax, i):        
        ax.plot(self.time_vector,
                self.optimal_forces[self.i_init:self.i_end,
                                    3*i:3*i+3])
        
        ax.set(
            xlabel = "time [s]",
            ylabel = "forces [N]",
            title = "optimal forces - " + self.foot_names[i],
        )
        ax.legend(["x-axis", "y-axis", "z-axis"], ncol=1)
        ax.set_xlim([self.time_init, self.time_end])
        
        
    def plot_optimal_torques(self, ax, i):        
        ax.plot(self.time_vector,
                self.optimal_torques[self.i_init:self.i_end,
                                     3*i:3*i+3])
        
        ax.set(
            xlabel = "time [s]",
            ylabel = "torques [Nm]",
            title = "optimal torques - " + self.foot_names[i],
        )
        ax.legend(["HAA", "HFE", "KFE"], ncol=3)
        ax.set_xlim([self.time_init, self.time_end])
        
        
    def plot_tangential_over_normal_optimal_forces(self, ax):
        """
        Plot the ratio of the tangential and normal component of the contact forces of the four feet.
        """
        
        tangential_over_normal_optimal_force = np.zeros((self.i_end - self.i_init, 4))
        
        for i in range(4):
            # + 1e-9 in order to avoid dividing by zero.
            tangential_over_normal_optimal_force[:,i] = \
                ( self.optimal_forces[self.i_init:self.i_end, 0+3*i]**2 + self.optimal_forces[self.i_init:self.i_end, 1+3*i]**2 )**0.5 / (self.optimal_forces[self.i_init:self.i_end, 2+3*i]**2 + 1e-9)
        
        ax.set(
            xlabel = "time [s]",
            ylabel = r"$f_t / f_n$",
            title = "ratio of the tangential and normal forces"
        )
        ax.legend(["LF", "RF", "LH", "RH"], ncol=2)
        ax.set_xlim([self.time_init, self.time_end])
        
        
    def plot_meas_vs_opti_forces(self, axs, i):
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
                title = "simulated vs optimized contact forces - " + self.dir_names[j] + "-axis" + " - " + self.foot_names[i]
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
            title = "simulated vs optimized contact forces - tangential - " + self.foot_names[i]
        )
        
        axs[3].legend(["simulated forces", "optimized forces"])
        axs[3].set_xlim([self.time_init, self.time_end])
            
    
    def plot_meas_vs_opti_depths(self, ax, i):
        if self.optimal_deformations.size > 0:
            k = int(self.optimal_deformations.shape[1] / 4)
            
            ax.plot(self.time_vector,
                    1000 * self.depths[self.i_init:self.i_end, i])
            
            ax.plot(self.time_vector,
                    1000 * self.optimal_deformations[self.i_init:self.i_end, k*i])
            
            ax.set(
                xlabel = "time [s]",
                ylabel = "depths [mm]",
                title = "measured vs optimized deformations - " + self.foot_names[i],
            )
            ax.legend(["measured", "optimized"])
        else:
            ax.set(title = "Not in my house!")
        
        
    def plot_meas_vs_opti_foot_positions(self, axs, i):
        #! Considers that all feet are in contact with the terrain
        
        for j in range(3):
            axs[j].plot(self.time_vector,
                        self.feet_positions[self.i_init:self.i_end, j + 3*i])
            
            axs[j].plot(self.time_vector,
                        self.feet_pos_meas[self.i_init:self.i_end, j + 3*i])
            
            axs[j].set(
                xlabel = "time [s]",
                ylabel = "position [m]",
                title = "true vs measured foot position - " + self.dir_names[j] + "-axis" + " - " + self.foot_names[i]
            )
            axs[j].legend(["true", "measured"])    
            axs[j].set_xlim([self.time_init, self.time_end])
            
        
    def plot_meas_vs_opti_feet_velocities(self, axs, i):
        for j in range(3):
            axs[j].plot(self.time_vector,
                        self.feet_pos_int[self.i_init:self.i_end, j + 3*i] - self.feet_pos_meas[self.i_init:self.i_end, j + 3*i])
            
            #! +1 if previously k = [1]
            axs[j].plot(self.time_vector,
                        self.feet_pos_meas[self.i_init+1:self.i_end+1, j + 3*i] - self.feet_pos_meas[self.i_init:self.i_end, j + 3*i])
            
            axs[j].set(
                xlabel = "time [s]",
                ylabel = "temp [m]",
                title = "measured vs optimized foot velocity - " + self.dir_names[j] + "-axis" + " - " + self.foot_names[i]
            )
            axs[j].legend(["measured", "optimized"])    
            axs[j].set_xlim([self.time_init, self.time_end])
            
        
    def plot_meas_vs_opti_feet_accelerations(self, axs, i):
        for j in range(3):
            axs[j].plot(self.time_vector,
                        self.feet_vel_int[self.i_init:self.i_end, j + 3*i] - self.feet_pos_meas[self.i_init:self.i_end, j + 3*i])
            
            #! +1 if previously k = [1]
            axs[j].plot(self.time_vector,
                        self.feet_vel_meas[self.i_init+1:self.i_end+1, j + 3*i] - self.feet_pos_meas[self.i_init:self.i_end, j + 3*i])
            
            axs[j].set(
                xlabel = "time [s]",
                ylabel = "temp [m\s]",
                title = "measured vs optimized foot acceleration - " + self.dir_names[j] + "-axis" + " - " + self.foot_names[i]
            )
            axs[j].legend(["measured", "optimized"])    
            axs[j].set_xlim([self.time_init, self.time_end])
            
    
    def plot_slippage(self, ax):
        ax.plot(self.time_vector, self.slippage[self.i_init:self.i_end])
        
        ax.set(
            xlabel = "time [s]",
            ylabel = "slippage [m]",
            title = "slippage",
        )
        ax.set_xlim([self.time_init, self.time_end])
        
        
    def plot_normalized_slippage(self, ax):
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
            
            
            
    def megaplot(self):
        feet_names = ["LF", "RF", "LH", "RH"]
        
        os.makedirs(self.foldername + '/' + self.format, exist_ok=True)
        
        for i in range(len(feet_names)):
            fig, axs = plt.subplots(4, 4, figsize=[3.5*self.x_size_def, 2.5*self.y_size_def])
        
            self.plot_meas_vs_opti_forces(axs[0:4, 0], i)
            self.plot_meas_vs_opti_depths(axs[0,1], i)
            self.plot_optimal_deformations(axs[1,1], i)
            self.plot_optimal_forces(axs[2,1], i)
            self.plot_optimal_torques(axs[3,1], i)
            self.plot_meas_vs_opti_foot_positions(axs[0:3,2], i)
            self.plot_meas_vs_opti_feet_velocities(axs[0:3,3], i)
            
            plt.savefig(self.foldername + '/' + self.format + '/' + "megaplot_" + feet_names[i] + '.' + self.format, bbox_inches="tight", format=self.format)
            plt.close(fig)
            
        # ==================================================================== #
            
        fig, axs = plt.subplots(4, 4, figsize=[3.5*self.x_size_def, 2.5*self.y_size_def])
        for i in range(len(feet_names)):
            self.plot_meas_vs_opti_forces(axs[0:4, i], i)
            
        plt.savefig(self.foldername + '/' + self.format + '/' + "meas_vs_opti_forces" + '.' + self.format, bbox_inches="tight", format=self.format)
        plt.close(fig)
        
        # ==================================================================== #
        
        fig, axs = plt.subplots(2, 1, figsize=[self.x_size_def, 2*self.y_size_def])
        self.plot_slippage(axs[0])
        self.plot_normalized_slippage(axs[1])
        
        plt.savefig(self.foldername + '/' + self.format + '/' + 'slippage' + '.' + self.format, bbox_inches="tight", format=self.format)
        plt.close(fig)
            
        
        
        
def main():
    default_cycler = (cycler(color=['#0072BD', '#D95319', '#EDB120', '#7E2F8E']) + 
                    cycler('linestyle', ['-', '--', '-', '--']))

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
    
    plot = Plot()
    plot.megaplot()
        
        
        
if __name__ == '__main__':
    main()