#include <array>
#include <cmath>
#include <functional>
#include <iostream>
#include <fstream>
#include <vector>
#include <atomic>
#include <algorithm>

#include <Eigen/Dense>

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>
#include <franka/gripper.h>

#include "examples_common.h"

double variable_control_formula(double velocity, double damping_min, double damping_max){
  return std::max(damping_min, (damping_max - (damping_max - damping_min) * fabs(velocity * velocity) * 4));
  // return std::max(damping_min, (damping_max - (damping_max - damping_min) * fabs(velocity) * 2));
}

struct data_collect {
    double time;
    Eigen::Vector3d trajectory_collect;
    double damping_d_x;
    double damping_d_y;
    double damping_d_z;
};

std::vector<data_collect> trajectory_data;
// save the memory before the control loop
// const size_t max_data_points = 10000;
// trajectory_data.reserve(max_data_points);

int main(int argc, char** argv) {
  // Check whether the required arguments were passed
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
    return -1;
  }

  try {
    franka::Robot robot(argv[1]);
    // {{0, 0, -M_PI_2, -M_PI_2, 0, M_PI_2, M_PI_4}};
    // {{-M_PI_2, 0, 0, -M_PI_2, 0, M_PI_2, M_PI_4}}
    std::array<double, 7> q_goal = {{-M_PI_2, 0, 0, -M_PI_2, 0, M_PI_2, M_PI_4}};
    MotionGenerator motion_generator(0.3, q_goal);
    std::cout << "WARNING: This will move the robot!" << std::endl
              << "Please make sure to have the user stop button at hand!" << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    robot.control(motion_generator);

    franka::Gripper gripper(argv[1]);
    gripper.homing();
    gripper.grasp(0.005, 0.1, 60);

    std::cout << "Finished moving to initial joint configuration." << std::endl;

    robot.setCollisionBehavior({{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                               {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                               {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                               {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});

    franka::Model model = robot.loadModel();
    franka::RobotState initial_state = robot.readOnce();
    Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
    Eigen::Vector3d initial_position(initial_transform.translation());
    Eigen::Quaterniond initial_orientation(initial_transform.linear());

    std::array<double, 42> Jacobian_array;
    std::array<double, 49> Mass_array;
    std::array<double, 7> Coriolis_array;
    std::array<double, 7> Gravity_array;
    Eigen::Matrix<double, 6, 7> Jacobian;
    std::array<double, 7> torque_control_array;
    Eigen::VectorXd torque_control;
    franka::Torques motion_finished_torques{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};

    double stiffness = 200;
    double velocity_x;
    double velocity_y;
    double velocity_z;
    double damping_min_x = 1;
    double damping_min_y = 1;
    double damping_min_z = 1;
    double damping_max_x = 10;
    double damping_max_y = 10;
    double damping_max_z = 10;
    // task_one 20 15 10 task_two_thress 10 7.5
    double damping_d_x = 10;
    double damping_d_y = 10;
    double damping_d_z = 10;
    double damping_d_array[3];
    Eigen::Matrix<double, 6, 1> Jacobian_dq;
    Eigen::Matrix<double, 7, 1> error_q;
    Eigen::Matrix<double, 7, 1> q_d_in_error_q;
    Eigen::Matrix<double, 7, 1> ddq;
    Eigen::Matrix<double, 6, 1> error;
    Eigen::Vector3d damping_d_vector;
    Eigen::Matrix3d damping_d_corner_matrix;
    Eigen::Matrix<double, 6, 6> Damping_d;
    Damping_d.setZero();
    Damping_d.bottomRightCorner(3, 3)= 10 * Eigen::MatrixXd::Identity(3, 3);

    double time = 0;
    static size_t count = 0;
    const size_t data_collect_frequency = 10;

    std::function<franka::Torques(const franka::RobotState&, franka::Duration)> 
        variable_impedance_control_callback = [&](const franka::RobotState& current_state, franka::Duration duration) -> franka::Torques {

        Eigen::Affine3d current_transform(Eigen::Matrix4d::Map(current_state.O_T_EE.data()));
        Eigen::Vector3d current_position(current_transform.translation());
        Eigen::Quaterniond current_orientation(current_transform.linear());

        if (count % data_collect_frequency == 0) {
        Eigen::Vector3d trajectory_collect = current_transform.translation();
        if (trajectory_data.size() < 10000){
        trajectory_data.push_back({time, trajectory_collect, damping_d_x, damping_d_y, damping_d_z});
        }
        }

        Jacobian_array = model.zeroJacobian(franka::Frame::kEndEffector, current_state);
        Mass_array = model.mass(current_state);
        Coriolis_array = model.coriolis(current_state);
        Gravity_array = model.gravity(current_state);

        Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(current_state.q.data());
        Eigen::Map<const Eigen::Matrix<double, 7, 1>> q_d(current_state.q_d.data());
        Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(current_state.dq.data());
        Eigen::Map<const Eigen::Matrix<double, 6, 7>> Jacobian(Jacobian_array.data());
        Eigen::Map<const Eigen::Matrix<double, 7, 7>> Mass(Mass_array.data());
        Eigen::Map<const Eigen::Matrix<double, 7, 1>> Coriolis(Coriolis_array.data());
        Eigen::Map<const Eigen::Matrix<double, 7, 1>> Gravity(Gravity_array.data());

        Jacobian_dq = Jacobian * dq;

        velocity_x = Jacobian_dq(0);
        velocity_y = Jacobian_dq(1);
        velocity_z = Jacobian_dq(2);

        damping_d_x = variable_control_formula(velocity_x, damping_min_x, damping_max_x);
        damping_d_y = variable_control_formula(velocity_y, damping_min_y, damping_max_y);
        damping_d_z = variable_control_formula(velocity_z, damping_min_z, damping_max_z);
        damping_d_array[0] = damping_d_x;
        damping_d_array[1] = damping_d_y;
        damping_d_array[2] = damping_d_z;
        damping_d_vector = Eigen::Vector3d(damping_d_array);
        damping_d_corner_matrix = damping_d_vector.asDiagonal();
        Damping_d.topLeftCorner(3, 3) = damping_d_corner_matrix;

        if (initial_orientation.coeffs().dot(current_orientation.coeffs()) < 0.0) {
          current_orientation.coeffs() << -current_orientation.coeffs();
          }
        Eigen::Quaterniond error_quaternion(current_orientation.inverse() * initial_orientation);
        error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
        error.tail(3) << -current_transform.linear() * error.tail(3);

        /*q_d_in_error_q = q;
        q_d_in_error_q(0, 0) = 0;
        q_d_in_error_q(1, 0) = 0;
        q_d_in_error_q(4, 0) = 0;
        error_q = q - q_d_in_error_q;*/

        torque_control = /*Mass * ddq +*/ Coriolis
                            - Jacobian.transpose() * Damping_d * (Jacobian * dq)
                            - Jacobian.transpose() * stiffness * error
                          /*- stiffness * error_q*/
                          /*- Jacobian.transpose() * Inertia_d * Jacobian * ddq*/;
        
        Eigen::VectorXd::Map(&torque_control_array[0], 7) = torque_control;

        time += duration.toSec();
        count++;

      // if (time >= 5) {
      //     std::cout << "60 seconds elapsed. Exiting control loop." << std::endl;
      //     return franka::MotionFinished(motion_finished_torques);
      //     }
      
      // std::cout << "Mass * ddq " << std::endl;

        return torque_control_array;

      };

      // start real-time control loop
      std::cout << "WARNING: Collision thresholds are set to high values. "
                << "Make sure you have the user stop at hand!" << std::endl
                << "After starting try to push the robot and see how it reacts." << std::endl
                << "Press Enter to continue..." << std::endl;
      std::cin.ignore();
      robot.control(variable_impedance_control_callback/*, false, 1000.0*/);

    } catch (const franka::Exception& ex) {
      // print exception
      std::cout << ex.what() << std::endl;
    }

    std::ofstream file("position_trajectory.csv");
    for (const auto& data : trajectory_data) {
      file << data.time << "," << data.trajectory_collect.x() << ","
           << data.trajectory_collect.y() << "," << data.trajectory_collect.z() << ","
           << data.damping_d_x << "," << data.damping_d_y << "," << data.damping_d_z << "\n";
    }
    file.close();
    std::cout << "Trajectory data saved to position_trajectory.csv" << std::endl;

    return 0;
}