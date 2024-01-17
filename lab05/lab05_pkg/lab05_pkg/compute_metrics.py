from cProfile import label
import numpy as np
import matplotlib.pyplot as plt

def compute_metrics():
        # Load data from .npy files
        odom_data = np.load("/workspaces/vscode_ros2_workspace/src/lab05/lab05_pkg/saved_bag/odom.npy",allow_pickle=True)
        filter_data = np.load("/workspaces/vscode_ros2_workspace/src/lab05/lab05_pkg/saved_bag/filter.npy",allow_pickle=True)
        ground_truth_data = np.load("/workspaces/vscode_ros2_workspace/src/lab05/lab05_pkg/saved_bag/ground_truth.npy",allow_pickle=True)

        odom_data = np.transpose(odom_data)
        filter_data = np.transpose(filter_data)
        ground_truth_data = np.transpose(ground_truth_data)

        # Calculate metrics
        pos_error_f_x = filter_data[0] - ground_truth_data[0]
        pos_error_f_y = filter_data[1] - ground_truth_data[1]
        pos_error_f = np.sqrt(pos_error_f_x**2 + pos_error_f_y**2)
        ang_error_f = filter_data[2] - ground_truth_data[2]

        pos_error_d_x = odom_data[0] - ground_truth_data[0]
        pos_error_d_y = odom_data[1] - ground_truth_data[1]
        pos_error_d = np.sqrt(pos_error_d_x**2 + pos_error_d_y**2)
        ang_error_d = odom_data[2] - ground_truth_data[2]

        position_rmse_f_x = np.sqrt(np.mean((filter_data[0] - ground_truth_data[0])**2))
        position_rmse_f_y = np.sqrt(np.mean((filter_data[1] - ground_truth_data[1])**2))
        position_max_abs_error_f_x = np.max(np.abs(filter_data[0] - ground_truth_data[0]))
        position_max_abs_error_f_y = np.max(np.abs(filter_data[1] - ground_truth_data[1]))
        position_total_cumulative_error_f_x = np.sum(np.linalg.norm(filter_data[0] - ground_truth_data[0]))
        position_total_cumulative_error_f_y = np.sum(np.linalg.norm(filter_data[1] - ground_truth_data[1]))

        total_distance = np.sum(np.sqrt(np.diff(ground_truth_data[0])**2 + np.diff(ground_truth_data[1])**2))

        position_error_percentage_f_x = ((filter_data[0][len(filter_data[0])-1] - ground_truth_data[0][len(ground_truth_data[0])-1]) / total_distance) * 100.0
        position_error_percentage_f_y = ((filter_data[1][len(filter_data[1])-1] - ground_truth_data[1][len(ground_truth_data[1])-1]) / total_distance) * 100.0

        position_rmse_d_x = np.sqrt(np.mean((odom_data[0] - ground_truth_data[0])**2))
        position_rmse_d_y = np.sqrt(np.mean((odom_data[1] - ground_truth_data[1])**2))
        position_max_abs_error_d_x = np.max(np.abs(odom_data[0] - ground_truth_data[0]))
        position_max_abs_error_d_y = np.max(np.abs(odom_data[1] - ground_truth_data[1]))
        position_total_cumulative_error_d_x = np.sum(np.linalg.norm(odom_data[0] - ground_truth_data[0]))
        position_total_cumulative_error_d_y = np.sum(np.linalg.norm(odom_data[1] - ground_truth_data[1]))
        position_error_percentage_d_x = ((odom_data[0][len(odom_data[0])-1] - ground_truth_data[0][len(ground_truth_data[0])-1]) / total_distance) * 100.0
        position_error_percentage_d_y = ((odom_data[1][len(odom_data[1])-1] - ground_truth_data[1][len(ground_truth_data[1])-1]) / total_distance) * 100.0

        orientation_rmse_f = np.sqrt(np.mean((filter_data[2] - ground_truth_data[2])**2))
        orientation_max_abs_error_f = np.max(np.abs(filter_data[2] - ground_truth_data[2]))
        orientation_total_cumulative_error_f = np.sum(np.abs(filter_data[2] - ground_truth_data[2]))

        orientation_rmse_d = np.sqrt(np.mean((odom_data[2] - ground_truth_data[2])**2))
        orientation_max_abs_error_d = np.max(np.abs(odom_data[2] - ground_truth_data[2]))
        orientation_total_cumulative_error_d = np.sum(np.abs(odom_data[2] - ground_truth_data[2]))

        #print required values
        print(f'total_distance= {total_distance}')
        
        print(f'rmse_F_x: {position_rmse_f_x}, rmse_F_y: {position_rmse_f_y}') 
        print(f'max_error_F_x: {position_max_abs_error_f_x}, max_error_F_y: {position_max_abs_error_f_y}') 
        print(f'cum_F_x: {position_total_cumulative_error_f_x}, cum_F_y: {position_total_cumulative_error_f_y}')
        print(f'final_error_F_x: {position_error_percentage_f_x}, final_error_F_y: {position_error_percentage_f_y}')

        print(f'rmse_D_x: {position_rmse_d_x}, rmse_D_y: {position_rmse_d_y}') 
        print(f'max_error_D_x: {position_max_abs_error_d_x}, max_error_D_y: {position_max_abs_error_d_y}')
        print(f'cum_D_x: {position_total_cumulative_error_d_x}, cum_D_y: {position_total_cumulative_error_d_y}')
        print(f'final_error_D_x: {position_error_percentage_d_x}, final_error_D_y: {position_error_percentage_d_y}')

        print(f'rmse_f_yaw: {orientation_rmse_f}, max_Error_f_yaw: {orientation_max_abs_error_f}, cum_f_yaw: {orientation_total_cumulative_error_f}')
        print(f'rmse_d_yaw: {orientation_rmse_d}, max_Error_d_yaw: {orientation_max_abs_error_d}, cum_d_yaw: {orientation_total_cumulative_error_d}')


        ##plot
        t = np.linspace(0, len(odom_data[0])*10, len(odom_data[0]))

        plt.figure()
        plt.plot(odom_data[0], odom_data[1], label='odom')
        plt.plot(filter_data[0], filter_data[1], label='filter')
        plt.plot(ground_truth_data[0], ground_truth_data[1], label='ground_truth')
        plt.xlabel('x')
        plt.ylabel('y')
        plt.title('XY trajectory')
        plt.legend()

        plt.figure()
        plt.plot(t, odom_data[2], label='odom')
        plt.plot(t, filter_data[2], label='filter')
        plt.plot(t, ground_truth_data[2], label='ground_truth')
        plt.xlabel('time(ms)')
        plt.ylabel('yaw')
        plt.title('yaw timeseries')
        plt.legend()

        plt.figure()
        plt.plot(t, pos_error_d, label='odom')
        plt.plot(t, pos_error_f, label='filter')
        plt.title('position error timeseries')
        plt.xlabel('time(ms)')
        plt.ylabel('error')
        plt.legend()

        plt.figure()
        plt.plot(t, ang_error_d, label='odom')
        plt.plot(t, ang_error_f, label='filter')
        plt.title('orientation error timeseries')
        plt.xlabel('time(ms)')
        plt.ylabel('error')
        plt.legend()

        plt.show()

compute_metrics()


        