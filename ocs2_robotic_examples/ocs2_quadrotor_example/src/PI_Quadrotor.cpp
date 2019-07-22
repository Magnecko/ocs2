#include <cfenv>

#include <ocs2_core/Dimensions.h>
#include <ocs2_core/initialization/SystemOperatingPoint.h>
#include <ocs2_mpc/MPC_PI.h>
#include <ocs2_mpc/MPC_Settings.h>
#include <ocs2_robotic_tools/common/RobotInterfaceBase.h>
#include <ocs2_oc/pi_solver/PiSolver.hpp>

#include <ocs2_quadrotor_example/cost/QuadrotorCost.h>
#include <ocs2_quadrotor_example/definitions.h>
#include <ocs2_quadrotor_example/dynamics/QuadrotorSystemDynamics.h>
#include <ocs2_quadrotor_example/ros_comm/MPC_ROS_Quadrotor.h>

#include <ocs2_core/misc/loadEigenMatrix.h>
#include <ros/package.h>

int main(int argc, char* argv[]) {
  feenableexcept(FE_DIVBYZERO | FE_INVALID | FE_OVERFLOW);

  // task file
  if (argc <= 1) {
    throw std::runtime_error("No task file specified. Aborting.");
  }

  std::string taskFile = ros::package::getPath("ocs2_quadrotor_example") + "/config/" + std::string(argv[1]) + "/task.info";
  using dim_t = ocs2::Dimensions<ocs2::quadrotor::STATE_DIM_, ocs2::quadrotor::INPUT_DIM_>;

  // Dynamics
  using dynamics_t = ocs2::quadrotor::QuadrotorSystemDynamics;
  ocs2::quadrotor::QuadrotorParameters<double> quadrotorParameters;
  quadrotorParameters.loadSettings(taskFile);
  dynamics_t::Ptr dynamics(new dynamics_t(quadrotorParameters));

  // Initial, nominal and final state
  dim_t::state_vector_t xInit, xNominal, xFinal;
  ocs2::loadEigenMatrix(taskFile, "initialState", xInit);
  xNominal.setZero();
  ocs2::loadEigenMatrix(taskFile, "x_final", xFinal);

  // Cost function
  dim_t::state_matrix_t Q, QFinal;
  ocs2::loadEigenMatrix(taskFile, "Q", Q);
  ocs2::loadEigenMatrix(taskFile, "Q_final", QFinal);
  dim_t::input_matrix_t R;
  ocs2::loadEigenMatrix(taskFile, "R", R);
  dim_t::input_vector_t uNominal;
  uNominal.setZero();
  using cost_t = ocs2::quadrotor::QuadrotorCost;
  std::unique_ptr<cost_t> quadrotorCost(new cost_t(Q, R, xNominal, uNominal, QFinal, xFinal));

  // Constraints
  using constraint_t = ocs2::ConstraintBase<ocs2::quadrotor::STATE_DIM_, ocs2::quadrotor::INPUT_DIM_>;
  constraint_t quadrotorConstraint;

  // Initialization
  dim_t::input_vector_t initialInput;
  initialInput.setZero();
  initialInput(0) = quadrotorParameters.quadrotorMass_ * quadrotorParameters.gravity_;
  using operatingpoint_t = ocs2::SystemOperatingPoint<ocs2::quadrotor::STATE_DIM_, ocs2::quadrotor::INPUT_DIM_>;
  operatingpoint_t::Ptr operatingPt(new operatingpoint_t(xInit, initialInput));
  //! @todo Use this initialization in the PI solver

  double timeHorizon;
  dim_t::scalar_array_t partitioningTimes;
  size_t numPartitions;
  ocs2::RobotInterfaceBase<ocs2::quadrotor::STATE_DIM_, ocs2::quadrotor::INPUT_DIM_>::definePartitioningTimes(
      taskFile, timeHorizon, numPartitions, partitioningTimes, true);
  // optimization settings
  ocs2::MPC_Settings mpcSettings;
  mpcSettings.loadSettings(taskFile);
  ocs2::PI_Settings piSettings;
  piSettings.loadSettings(taskFile);

  ocs2::MPC_PI<ocs2::quadrotor::STATE_DIM_, ocs2::quadrotor::INPUT_DIM_> mpc_pi(dynamics, std::move(quadrotorCost), quadrotorConstraint,
                                                                                partitioningTimes, mpcSettings, piSettings);

  // cost desired trajectories
  using cost_desired_trajectories_t = ocs2::PiSolver<ocs2::quadrotor::STATE_DIM_, ocs2::quadrotor::INPUT_DIM_>::cost_desired_trajectories_t;
  double initTime, finalTime;
  initTime = 0.0;
  ocs2::loadScalar(taskFile, "mpcTimeHorizon.timehorizon", finalTime);
  cost_desired_trajectories_t::scalar_array_t desiredTimeArray{initTime, finalTime};
  cost_desired_trajectories_t::dynamic_vector_array_t desiredStateArray(2), desiredInputArray(2);
  desiredStateArray[0] = xFinal;
  desiredStateArray[1] = xFinal;
  desiredInputArray[0].setZero(ocs2::quadrotor::STATE_DIM_);
  desiredInputArray[1].setZero(ocs2::quadrotor::INPUT_DIM_);
  cost_desired_trajectories_t costDesiredTraj(desiredTimeArray, desiredStateArray, desiredInputArray);
  mpc_pi.setCostDesiredTrajectories(costDesiredTraj);
  ocs2::quadrotor::MPC_ROS_Quadrotor mpcNode(mpc_pi, "quadrotor");

  mpcNode.launchNodes(argc, argv);

  // Successful exit
  return 0;
}
