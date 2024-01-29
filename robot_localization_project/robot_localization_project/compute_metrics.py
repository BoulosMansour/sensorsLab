from cProfile import label
import numpy as np
import matplotlib.pyplot as plt
from utils import plot_covariance

def compute_metrics():
        # Load data from .npy files
        odom_data = np.load("/workspaces/vscode_ros2_workspace/src/robot_localization_project/saved_data/odom.npy",allow_pickle=True)
        ekf_vel_data = np.load("/workspaces/vscode_ros2_workspace/src/robot_localization_project/saved_data/ekf_vel.npy",allow_pickle=True)
        ekf_odom_data = np.load("/workspaces/vscode_ros2_workspace/src/robot_localization_project/saved_data/ekf_odom.npy",allow_pickle=True)
        ground_truth_data = np.load("/workspaces/vscode_ros2_workspace/src/robot_localization_project/saved_data/ground_truth.npy",allow_pickle=True)
        sim_prop = np.load("/workspaces/vscode_ros2_workspace/src/robot_localization_project/saved_data/sim_prop.npy", allow_pickle=True)
        ekf_vel_Sigma_prediction = np.load("/workspaces/vscode_ros2_workspace/src/robot_localization_project/saved_data/ekf_vel_Sigma_prediction.npy", allow_pickle=True)
        ekf_vel_Sigma_correction = np.load("/workspaces/vscode_ros2_workspace/src/robot_localization_project/saved_data/ekf_vel_Sigma_correction.npy", allow_pickle=True)      
        ekf_odom_Sigma_prediction = np.load("/workspaces/vscode_ros2_workspace/src/robot_localization_project/saved_data/ekf_odom_Sigma_prediction.npy", allow_pickle=True)
        ekf_odom_Sigma_correction = np.load("/workspaces/vscode_ros2_workspace/src/robot_localization_project/saved_data/ekf_odom_Sigma_correction.npy", allow_pickle=True) 

        odom_data = np.transpose(odom_data)
        ekf_vel_data = np.transpose(ekf_vel_data)
        ekf_odom_data = np.transpose(ekf_odom_data)
        ground_truth_data = np.transpose(ground_truth_data)

        # Calculate metrics
        pos_error_ekf_vel_x = ekf_vel_data[0] - ground_truth_data[0]
        pos_error_ekf_vel_y = ekf_vel_data[1] - ground_truth_data[1]
        pos_error_ekf_vel = np.sqrt(pos_error_ekf_vel_x**2 + pos_error_ekf_vel_y**2)
        ang_error_ekf_vel = ekf_vel_data[2] - ground_truth_data[2]

        pos_error_ekf_odom_x = ekf_odom_data[0] - ground_truth_data[0]
        pos_error_ekf_odom_y = ekf_odom_data[1] - ground_truth_data[1]
        pos_error_ekf_odom = np.sqrt(pos_error_ekf_odom_x**2 + pos_error_ekf_odom_y**2)
        ang_error_ekf_odom = ekf_odom_data[2] - ground_truth_data[2]

        pos_error_d_x = odom_data[0] - ground_truth_data[0]
        pos_error_d_y = odom_data[1] - ground_truth_data[1]
        pos_error_d = np.sqrt(pos_error_d_x**2 + pos_error_d_y**2)
        ang_error_d = odom_data[2] - ground_truth_data[2]

        position_rmse_ekf_vel_x = np.sqrt(np.mean((ekf_vel_data[0] - ground_truth_data[0])**2))
        position_rmse_ekf_vel_y = np.sqrt(np.mean((ekf_vel_data[1] - ground_truth_data[1])**2))
        position_max_abs_error_ekf_vel_x = np.max(np.abs(ekf_vel_data[0] - ground_truth_data[0]))
        position_max_abs_error_ekf_vel_y = np.max(np.abs(ekf_vel_data[1] - ground_truth_data[1]))
        position_total_cumulative_error_ekf_vel_x = np.sum(np.linalg.norm(ekf_vel_data[0] - ground_truth_data[0]))
        position_total_cumulative_error_ekf_vel_y = np.sum(np.linalg.norm(ekf_vel_data[1] - ground_truth_data[1]))

        position_rmse_ekf_odom_x = np.sqrt(np.mean((ekf_odom_data[0] - ground_truth_data[0])**2))
        position_rmse_ekf_odom_y = np.sqrt(np.mean((ekf_odom_data[1] - ground_truth_data[1])**2))
        position_max_abs_error_ekf_odom_x = np.max(np.abs(ekf_odom_data[0] - ground_truth_data[0]))
        position_max_abs_error_ekf_odom_y = np.max(np.abs(ekf_odom_data[1] - ground_truth_data[1]))
        position_total_cumulative_error_ekf_odom_x = np.sum(np.linalg.norm(ekf_odom_data[0] - ground_truth_data[0]))
        position_total_cumulative_error_ekf_odom_y = np.sum(np.linalg.norm(ekf_odom_data[1] - ground_truth_data[1]))

        total_distance = np.sum(np.sqrt(np.diff(ground_truth_data[0])**2 + np.diff(ground_truth_data[1])**2))

        position_error_percentage_ekf_vel_x = ((ekf_vel_data[0][len(ekf_vel_data[0])-1] - ground_truth_data[0][len(ground_truth_data[0])-1]) / total_distance) * 100.0
        position_error_percentage_ekf_vel_y = ((ekf_vel_data[1][len(ekf_vel_data[1])-1] - ground_truth_data[1][len(ground_truth_data[1])-1]) / total_distance) * 100.0

        position_error_percentage_ekf_odom_x = ((ekf_odom_data[0][len(ekf_odom_data[0])-1] - ground_truth_data[0][len(ground_truth_data[0])-1]) / total_distance) * 100.0
        position_error_percentage_ekf_odom_y = ((ekf_odom_data[1][len(ekf_odom_data[1])-1] - ground_truth_data[1][len(ground_truth_data[1])-1]) / total_distance) * 100.0


        position_rmse_d_x = np.sqrt(np.mean((odom_data[0] - ground_truth_data[0])**2))
        position_rmse_d_y = np.sqrt(np.mean((odom_data[1] - ground_truth_data[1])**2))
        position_max_abs_error_d_x = np.max(np.abs(odom_data[0] - ground_truth_data[0]))
        position_max_abs_error_d_y = np.max(np.abs(odom_data[1] - ground_truth_data[1]))
        position_total_cumulative_error_d_x = np.sum(np.linalg.norm(odom_data[0] - ground_truth_data[0]))
        position_total_cumulative_error_d_y = np.sum(np.linalg.norm(odom_data[1] - ground_truth_data[1]))
        position_error_percentage_d_x = ((odom_data[0][len(odom_data[0])-1] - ground_truth_data[0][len(ground_truth_data[0])-1]) / total_distance) * 100.0
        position_error_percentage_d_y = ((odom_data[1][len(odom_data[1])-1] - ground_truth_data[1][len(ground_truth_data[1])-1]) / total_distance) * 100.0

        orientation_rmse_ekf_vel = np.sqrt(np.mean((ekf_vel_data[2] - ground_truth_data[2])**2))
        orientation_max_abs_error_ekf_vel = np.max(np.abs(ekf_vel_data[2] - ground_truth_data[2]))
        orientation_total_cumulative_error_ekf_vel = np.sum(np.abs(ekf_vel_data[2] - ground_truth_data[2]))

        orientation_rmse_ekf_odom = np.sqrt(np.mean((ekf_odom_data[2] - ground_truth_data[2])**2))
        orientation_max_abs_error_ekf_odom = np.max(np.abs(ekf_odom_data[2] - ground_truth_data[2]))
        orientation_total_cumulative_error_ekf_odom = np.sum(np.abs(ekf_odom_data[2] - ground_truth_data[2]))

        orientation_rmse_d = np.sqrt(np.mean((odom_data[2] - ground_truth_data[2])**2))
        orientation_max_abs_error_d = np.max(np.abs(odom_data[2] - ground_truth_data[2]))
        orientation_total_cumulative_error_d = np.sum(np.abs(odom_data[2] - ground_truth_data[2]))

        #print required values
        print(f'simulation time= {sim_prop[0]} s')
        print(f'total_distance= {total_distance}')

        print()

        print(f'rmse_ekf_vel_x: {position_rmse_ekf_vel_x}, rmse_ekf_vel_y: {position_rmse_ekf_vel_y}') 
        print(f'max_error_ekf_vel_x: {position_max_abs_error_ekf_vel_x}, max_error_ekf_vel_y: {position_max_abs_error_ekf_vel_y}') 
        print(f'cum_ekf_vel_x: {position_total_cumulative_error_ekf_vel_x}, cum_ekf_vel_y: {position_total_cumulative_error_ekf_vel_y}')
        print(f'final_error_ekf_vel_x: {position_error_percentage_ekf_vel_x}, final_error_ekf_vel_y: {position_error_percentage_ekf_vel_y}')

        print()

        print(f'rmse_ekf_odom_x: {position_rmse_ekf_odom_x}, rmse_ekf_odom_y: {position_rmse_ekf_odom_y}') 
        print(f'max_error_ekf_odom_x: {position_max_abs_error_ekf_odom_x}, max_error_ekf_odom_y: {position_max_abs_error_ekf_odom_y}') 
        print(f'cum_ekf_odom_x: {position_total_cumulative_error_ekf_odom_x}, cum_ekf_odom_y: {position_total_cumulative_error_ekf_odom_y}')
        print(f'final_error_ekf_odom_x: {position_error_percentage_ekf_odom_x}, final_error_ekf_odom_y: {position_error_percentage_ekf_odom_y}')
        
        print()

        print(f'rmse_D_x: {position_rmse_d_x}, rmse_D_y: {position_rmse_d_y}') 
        print(f'max_error_D_x: {position_max_abs_error_d_x}, max_error_D_y: {position_max_abs_error_d_y}')
        print(f'cum_D_x: {position_total_cumulative_error_d_x}, cum_D_y: {position_total_cumulative_error_d_y}')
        print(f'final_error_D_x: {position_error_percentage_d_x}, final_error_D_y: {position_error_percentage_d_y}')

        print()

        print(f'rmse_ekf_vel_yaw: {orientation_rmse_ekf_vel}, max_Error_ekf_vel_yaw: {orientation_max_abs_error_ekf_vel}, cum_ekf_vel_yaw: {orientation_total_cumulative_error_ekf_vel}')
        print(f'rmse_ekf_odom_yaw: {orientation_rmse_ekf_odom}, max_Error_ekf_odom_yaw: {orientation_max_abs_error_ekf_odom}, cum_ekf_odom_yaw: {orientation_total_cumulative_error_ekf_odom}')
        print(f'rmse_d_yaw: {orientation_rmse_d}, max_Error_d_yaw: {orientation_max_abs_error_d}, cum_d_yaw: {orientation_total_cumulative_error_d}')

        ## plot
        t = np.linspace(0, sim_prop[0], num= len(odom_data[0]))

        #covariance
        steps = len(odom_data[0])
        ellipse_step_s = 5.0
        ellipse_step = int(ellipse_step_s / sim_prop[2])
        
        plt.figure()
        plt.plot(odom_data[0], odom_data[1], label='odom')
        plt.plot(ekf_vel_data[0], ekf_vel_data[1], label='ekf_vel')
        plt.plot(ekf_odom_data[0], ekf_odom_data[1], label='ekf_odom')
        plt.plot(ground_truth_data[0], ground_truth_data[1], label='ground_truth')
        plt.xlabel('x')
        plt.ylabel('y')
        plt.title('XY trajectory')
        plt.legend()

        plt.figure()
        added_legend_prediction = False
        added_legend_correction = False
        for i in range(steps):
                if i % ellipse_step == 0:  # plot the prior covariance ellipses every ellipse_step_s seconds
                        plot_covariance(
                        (ekf_vel_data[0][i], ekf_vel_data[1][i]),
                        (ekf_vel_Sigma_prediction[i][0:2, 0:2])/100,
                        std=6,
                        facecolor="k",
                        alpha=0.4,
                        label="Predicted Cov" if not added_legend_prediction else "",
                        )
                        plot_covariance(
                        (ekf_vel_data[0][i], ekf_vel_data[1][i]),
                        (ekf_vel_Sigma_correction[i][0:2, 0:2])/100,
                        std=6,
                        facecolor="g",
                        alpha=0.8,
                        label="Corrected Cov" if not added_legend_correction else "",
                        )
                        added_legend_prediction = True
                        added_legend_correction = True
        plt.plot(ekf_vel_data[0], ekf_vel_data[1], label='ekf_vel')
        plt.xlabel('x')
        plt.ylabel('y')
        plt.title('ekf_vel_covariance')
        plt.legend()

        plt.figure()
        added_legend_prediction = False
        added_legend_correction = False
        for i in range(steps):
                if i % ellipse_step == 0:  # plot the prior covariance ellipses every ellipse_step_s seconds
                        plot_covariance(
                        (ekf_odom_data[0][i], ekf_odom_data[1][i]),
                        (ekf_odom_Sigma_prediction[i][0:2, 0:2])/100,
                        std=6,
                        facecolor="k",
                        alpha=0.4,
                        label="Predicted Cov" if not added_legend_prediction else "",
                        )
                        plot_covariance(
                        (ekf_odom_data[0][i], ekf_odom_data[1][i]),
                        (ekf_odom_Sigma_correction[i][0:2, 0:2])/100,
                        std=6,
                        facecolor="g",
                        alpha=0.8,
                        label="Corrected Cov" if not added_legend_correction else "",
                        )
                        added_legend_prediction = True
                        added_legend_correction = True
        plt.plot(ekf_odom_data[0], ekf_odom_data[1], label='ekf_vel')
        plt.xlabel('x')
        plt.ylabel('y')
        plt.title('ekf_odom_covariance')
        plt.legend()

        plt.figure()
        plt.plot(t, odom_data[2], label='odom')
        plt.plot(t, ekf_vel_data[2], label='ekf_vel')
        plt.plot(t, ekf_odom_data[2], label='ekf_odom')
        plt.plot(t, ground_truth_data[2], label='ground_truth')
        plt.xlabel('time(s)')
        plt.ylabel('yaw')
        plt.title('yaw timeseries')
        plt.legend()

        plt.figure()
        plt.plot(t, pos_error_d, label='odom')
        plt.plot(t, pos_error_ekf_vel, label='ekf_vel')
        plt.plot(t, pos_error_ekf_odom, label='ekf_odom')
        plt.title('position error timeseries')
        plt.xlabel('time(s)')
        plt.ylabel('error')
        plt.legend()

        plt.figure()
        plt.plot(t, ang_error_d, label='odom')
        plt.plot(t, ang_error_ekf_vel, label='ekf_vel')
        plt.plot(t, ang_error_ekf_odom, label='ekf_odom')
        plt.title('orientation error timeseries')
        plt.xlabel('time(s)')
        plt.ylabel('error')
        plt.legend()

        plt.show()

compute_metrics()


        