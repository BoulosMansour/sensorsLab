from utils import plot_covariance
import matplotlib.pyplot as plt
from robot_localization_project.ekf import RobotEKF
from robot_localization_project.measurement_model import z_landmark, residual
import numpy as np

def run_localization(
    ekf: RobotEKF,
    ekf_dt,
    landmarks,
    std_lin_vel,
    std_ang_vel,
    std_range,
    std_bearing,
    sim_step_s=0.1,
    ellipse_step_s=5.0,
    sim_length_s=1,
    eval_hx= None,
):
    sim_pos = ekf.mu.copy()  # simulated position, copy the initial position set inside the EKF
    odom_pos = ekf.mu.copy()  # odometry position, copy the initial position set inside the EKF
    sim_noise_generator = np.random.default_rng(42424242)  # random noise generator

    cmd_vel = np.array(
        [[1.10, 0.05]]
    ).T  # velocity command (v, omega). In this case will be constant for the whole simulation

    # convert the durations to number of time steps
    steps = int(sim_length_s / sim_step_s)
    ekf_step = int(ekf_dt / sim_step_s)
    ellipse_step = int(ellipse_step_s / sim_step_s)

    # Initialize a plot and insert the landmarks
    fig, ax = plt.subplots(1, 2, figsize=(9, 4))
    lmarks_legend = ax[0].scatter(landmarks[:, 0], landmarks[:, 1], marker="s", s=60, label="Landmarks")

    track = []  # list to store all the robot positions
    track_odom = []  # list to store all the odometry positions
    track_ekf = [ekf.mu.copy()]  # list to store all the ekf positions

    # The main loop that runs the simulation
    for i in range(steps):
        # to simulate the error in the actuation of the robot, we add some Gaussian noise to the velocity command
        noisy_vel = cmd_vel + np.array([[std_lin_vel, std_ang_vel]]).T * sim_noise_generator.normal(size=(2, 1))
        # Simulate robot motion for sim_step_s seconds using the Velocity Motion Model.
        # Complete with the correct data to evaluate the motion model, use the noisy_vel variable
        sim_pos = ekf.eval_gux(x = sim_pos.flatten()[0], y= sim_pos.flatten()[1], theta=sim_pos.flatten()[2], v= noisy_vel.flatten()[0], w= noisy_vel.flatten()[1], dt=sim_step_s) 
        track.append(sim_pos)

        # to simulate the error in the odometry reading, we take another Gaussian sample of the velocity command
        noisy_vel = cmd_vel + np.array([[std_lin_vel, std_ang_vel]]).T * sim_noise_generator.normal(size=(2, 1))
        # complete with the correct data to evaluate the motion model, use the noisy_vel variable
        odom_pos = ekf.eval_gux(x = odom_pos.flatten()[0], y= odom_pos.flatten()[1], theta=odom_pos.flatten()[2], v= noisy_vel.flatten()[0], w= noisy_vel.flatten()[1], dt=sim_step_s)
        track_odom.append(odom_pos)

        if i % ekf_step == 0 and i != 0:  # only update ekf at dt intervals
            # run the prediction step of the EKF
            ekf.predict(u=cmd_vel.flatten(), g_extra_args=(ekf_dt))

            if i % ellipse_step == 0:  # plot the prior covariance ellipses every ellipse_step_s seconds
                pri_ellipse = plot_covariance(
                    (ekf.mu[0, 0], ekf.mu[1, 0]),
                    ekf.Sigma[0:2, 0:2],
                    std=6,
                    facecolor="k",
                    alpha=0.4,
                    label="Predicted Cov",
                    ax=ax[0],
                )

            for lmark in landmarks:  # loop over each landmark
                z = z_landmark(sim_pos, lmark, std_range, std_bearing, eval_hx)  # simulate the measurement of the landmark

                # if any landmark detected by the sensor, update the EKF
                if z is not None:
                    # run the correction step of the EKF
                    ekf.update(z, lmark, residual=residual)

            if i % ellipse_step == 0:  # plot the posterior covariance ellipses every ellipse_step_s seconds
                post_ellipse = plot_covariance(
                    (ekf.mu[0, 0], ekf.mu[1, 0]),
                    ekf.Sigma[0:2, 0:2],
                    std=6,
                    facecolor="g",
                    alpha=0.8,
                    label="Corrected Cov",
                    ax=ax[0],
                )
            track_ekf.append(ekf.mu.copy())

    # draw plots
    track = np.array(track)
    track_odom = np.array(track_odom)
    track_ekf = np.array(track_ekf)
    
    # trajectory plots
    (track_legend,) = ax[0].plot(track[:, 0], track[:, 1], label="Real robot path")
    (track_odom_legend,) = ax[0].plot(track_odom[:, 0], track_odom[:, 1], "--", label="Odometry path")
    ax[0].axis("equal")
    ax[0].set_title("EKF Robot localization")
    ax[0].legend(handles=[lmarks_legend, track_legend, track_odom_legend, pri_ellipse, post_ellipse])

    # error plots
    ekf_err, =  ax[1].plot(
        np.arange(0, sim_length_s, ekf_dt), 
        np.linalg.norm(track[::ekf_step, :2] - track_ekf[:, :2], axis=1), 
        '-o',
        label="EKF error",
    )
    odom_err, = ax[1].plot(
        np.arange(0, sim_length_s, sim_step_s), 
        np.linalg.norm(track[:, :2] - track_odom[:, :2], axis=1), 
        label="Odometry error",
    )
    ax[1].legend(handles=[ekf_err, odom_err])
    ax[1].set_title("Robot path error")

    fig.suptitle("EKF Robot localization, Velocity Motion Model")
    fig.tight_layout()

    plt.show()