from cycler import cycler
import matplotlib.pyplot as plt
import numpy as np



def plot():
    default_cycler = (cycler(color=['#0072BD', '#D95319', '#EDB120', '#7E2F8E']) + 
                    cycler('linestyle', ['-', '--', '-', '--']))

    textsize = 16

    plt.rc('font', family='serif', serif='Times')
    plt.rc('text', usetex=True)
    plt.rc('xtick', labelsize=textsize)
    plt.rc('ytick', labelsize=textsize)
    plt.rc('axes', labelsize=24, prop_cycle=default_cycler)
    plt.rc('legend', fontsize=textsize)


    filenames = ['optimal_deformations',
                'optimal_forces',
                'optimal_torques',
                'orientation_error',
                'position_error',
                'slippage',
                'velocity_error',
                'contact_forces'
                ]

    foldernames = ['log']

    time_init = 0
    time_end = 10

    for foldername in foldernames:

        time_vector = np.loadtxt(foldername + '/csv/' + 'time' + '.csv', delimiter=",")

        for i in range(len(time_vector)):
            if time_vector[i] >= time_init:
                i_init = i
                break
            
        for i in range(len(time_vector)-1, -1, -1):
            if time_vector[i] <= time_end:
                i_end = i
                break

        for filename in filenames:
            
            if filename == "contact_forces":
                contact_forces = np.loadtxt(foldername + '/csv/' + filename + '.csv', delimiter=",")
                optimal_forces = np.loadtxt(foldername + '/csv/' + "optimal_forces" + '.csv', delimiter=",")
                
                parts = ['_LF', '_RH']
                parts_indexes = [0, 3, 6, 9]
                
                for i in range(len(parts)):
                    fig, axs = plt.subplots(3)
                    for j in range(3):
                        axs[j].plot(time_vector[i_init:i_end], contact_forces[i_init:i_end, j + parts_indexes[2*i]])
                        axs[j].plot(time_vector[i_init:i_end], optimal_forces[i_init:i_end, j + parts_indexes[2*i]])
                        
                        dir_names = ["x", "y", "z"]
                        axs[j].set(
                            xlabel = "time [s]",
                            ylabel = "force [N]",
                            title = dir_names[i] + "-axis"
                        )
                        
                        axs[j].legend(["simulated forces", "optimized forces"])
                        
                    plt.xlim([time_init, time_end])
                    plt.grid(linestyle='dotted', linewidth = 0.25)
                    plt.savefig(foldername + '/pdf/' + filename + parts[i] + '.pdf', bbox_inches="tight", format='pdf')
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
                
            if filename in ['optimal_deformations', 'optimal_forces', 'optimal_torques']:
                parts = ['_LF', '_RH']
                parts_indexes = [0, 3, 6, 9]
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

                plt.grid(linestyle='dotted', linewidth = 0.25)
                    
                plt.savefig(foldername + '/pdf/' + filename + parts[i] + '.pdf', bbox_inches="tight", format='pdf')

                plt.close(fig)
                
            if filename == 'optimal_forces':
                tangential_over_normal_optimal_force = np.zeros((i_end - i_init, 4))
                
                arr2  = arr[i_init:i_end,:]
                
                for i in range(4):
                    tangential_over_normal_optimal_force[:,i] = (arr2[:,0+3*i]**2 + arr2[:,1+3*i]**2)**0.5 / (arr2[:,2+3*i] + 1e-9)
                    
                plt.plot(time_vector[i_init:i_end], tangential_over_normal_optimal_force)
                
                plt.ylabel(r"$f_{t} / f_{n}$")
                plt.xlabel("time [s]")
                plt.legend(["LF", "RF", "LH", "RH"], ncol=2)
                
                plt.xlim([time_init, time_end])
                plt.grid(linestyle='dotted', linewidth = 0.25)
                plt.savefig(foldername + '/pdf/' + 'tangential_over_normal_optimal_force' + '.pdf', bbox_inches="tight", format='pdf')
                plt.close(fig)
                
                
                
if __name__ == '__main__':
    plot()