/**
 * @file main.cpp
 * @authors Lorenzo Moretti, Ines Sorrentino
 * @copyright 2024 Istituto Italiano di Tecnologia (IIT). This software may be
 * modified and distributed under the terms of the BSD-3 clause license.
 */

#ifndef MOMENTUM_BASED_TORQUE_CONTROL_CONTROLLER_H
#define MOMENTUM_BASED_TORQUE_CONTROL_CONTROLLER_H

#include <chrono>
#include <cstddef>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include <iDynTree/KinDynComputations.h>

#include <BipedalLocomotion/FloatingBaseEstimators/LeggedOdometry.h>
#include <BipedalLocomotion/Math/Constants.h>
#include <BipedalLocomotion/Math/Wrench.h>
#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/RobotInterface/YarpHelper.h>
#include <BipedalLocomotion/RobotInterface/YarpRobotControl.h>
#include <BipedalLocomotion/RobotInterface/YarpSensorBridge.h>
#include <BipedalLocomotion/System/Sink.h>
#include <BipedalLocomotion/YarpUtilities/VectorsCollectionServer.h>

#include <MomentumBasedTorqueControl/QPproblem.h>
#include <MomentumBasedTorqueControl/ReferenceGenerator.h>

namespace MomentumBasedController {

class WholeBodyController : public BipedalLocomotion::System::Sink<
                                BipedalLocomotion::System::EmptySignal> {

  bool m_isFirstStep{true}; /**< true if the controller is running for the first
                               time. */

  int m_numberOfConsecutiveQPFailures{
      0}; /**< Number of consecutive QP failures. */

  std::shared_ptr<iDynTree::KinDynComputations>
      m_kinDyn; /**< KinDynComputations object. */

  std::vector<std::string> m_jointsList; /**< List of controlled joints. */
  size_t m_numberOfJoints;               /**< Number of controlled joints. */

  WholeBodyControllerReference
      m_references; /**< References for the controller. */

  double m_totalMass; /**< Total mass of the robot. */

  Eigen::VectorXd m_gravityWrench; /**< Gravity 6D wrench. */

  Eigen::VectorXd m_jointsPosition;      /**< Measured joints position. */
  Eigen::VectorXd m_jointsVelocity;      /**< Measured joints velocity. */
  Eigen::VectorXd m_jointsDesiredTorque; /**< Desired joints torque. */

  manif::SE3d
      m_baseTransform; /**< Pose of the base (rootLink) in the world frame. */
  manif::SE3d::Tangent
      m_baseVelocity; /**< 6D Velocity of the base in the world frame. */

  manif::SE3d m_worldLeftFootTransform; /**< Pose of the left foot in the world
                                           frame. */

  manif::SE3d m_worldRightFootTransform; /**< Pose of the right foot in the
                                            world frame. */

  Eigen::Vector3d
      m_comPosition; /**< Center of mass position in the world frame. */
  Eigen::Vector3d m_comVelocity;        /**< Center of mass velocity */
  Eigen::Vector3d m_comPositionInitial; /**< Initial center of mass position. */

  struct Regularizers {
    double hessianQP{1e-7};
    double tolerancePinv{1e-5};
    double dampingFactorPinvLambda{1};
    double dampingFactorPinvBaseVel{1e-7};
  };

  Regularizers m_regularizers; /**< Regularizers of the controller. */

  struct Gains {
    Eigen::Vector3d kpAngularMomentum;
    Eigen::Vector3d kiAngularMomentum;
    Eigen::Vector3d kpCoM;
    Eigen::Vector3d kdCoM;
    Eigen::VectorXd kpPostural;
    Eigen::VectorXd kdPostural;
  };

  Gains m_gains; /**< Gains of the controller. */

  QPproblem m_QPproblem; /**< QP problem object. */
  const size_t m_numberOfVariablesPerFoot{
      6}; /**< Number of optimization variables per foot. */
  size_t m_numberOfConstraintsPerFoot{
      7}; /**< Number of constraints per foot. */

  struct Settings {
    bool fallbackToUnconstrainedQP{
        true}; /**< If true, solve the unconstrained version of the original QP,
                  if the original QP fails. */
    int maxNumberConsecutiveQPFailures{
        -1}; /**< Maximum number of consecutive QP failures and consequent
               resolution of the unconstrained QP, after which stop the
               controller. If set to -1, no check of consecutive failures is
               performed. */
    bool saturateTorques{true}; /**< If true, saturate the torques. */
    double maxTorque{150.0};    /**< Torque saturation value. */
  };

  Settings m_settings; /**< Settings of the controller. */

  struct RigidContactConstraints {
    double staticFrictionCoefficient{0.333}; /**< The static linear coefficient
                                                of friction. */
    double torsionalFrictionCoefficient{0.013}; /**< The torsional coefficient
                                                   of friction. */
    double fZmin{10.0}; /**< The minimum positive vertical force at contact. */
    size_t numberOfPoints{4}; /**< Number of points in each quadrant for
                                 linearizing the friction cone. */
    Eigen::Matrix2d contactAreaSize{
        {-0.1, 0.1},
        {-0.05, 0.05}}; /**< the physical size of the contact area, defined as a
                           2x2 matrix arranged as [-x, x; -y, y]*/

    Eigen::MatrixXd constraintsMatrixLeftFoot;
    Eigen::VectorXd constraintsVectorLeftFoot;
    Eigen::MatrixXd constraintsMatrixRightFoot;
    Eigen::VectorXd constraintsVectorRightFoot;
  };

  RigidContactConstraints m_rigidContactConstraints;

  struct CentroidalDynamics {
    Eigen::MatrixXd A;      /**< Matrix mapping the forces and moments into the
                                centroidal momentum rate. */
    Eigen::MatrixXd pinvA;  /**< Pseudo-inverse of the matrix A. */
    Eigen::MatrixXd A_left; /**< Matrix mapping the forces and moments acting
                               on the left foot into the centroidal momentum
                               rate. */
    Eigen::MatrixXd nullSpaceProjectorA; /**< Null space projector of the matrix
                                            A. */
    Eigen::MatrixXd A_right;   /**< Matrix mapping the forces and moments acting
                                    on the right foot into the centroidal momentum
                                    rate. */
    Eigen::VectorXd L_DotStar; /**< Desired centroidal momentum rate. */
    Eigen::VectorXd f_LDotStar; /**< Contact forces and moments required to get
                                   L_DotStar. */
  };

  CentroidalDynamics m_centroidalDynamics; /**< Centroidal dynamics. */

  struct FullDynamics {
    Eigen::MatrixXd
        massMatrix; /**< Mass matrix which can be partitioned as
follows:
M = [ Mb ,   Mbs
     Mbs',    Ms ];
where: Mb  \in R^{6 x 6}, Mbs in R^{6 x 6+NDOF}, Ms  \in R^{NDOF x NDOF} */
    Eigen::MatrixXd
        massMatrixB; /**< portion of Mass matrix related to the base. */
    Eigen::MatrixXd massMatrixS;  /**< portion of Mass matrix related to the
                                     joints. */
    Eigen::MatrixXd massMatrixBS; /**< portion of Mass matrix related to the
                                    base and joints. */
    Eigen::MatrixXd
        massMatrixBar; /**< Mbar is the mass matrix associated with the joint
                          dynamics, i.e. Mbar = Ms - Mbs'/Mb*Mbs; */

    Eigen::VectorXd h;              /**< Generalized bias forces. */
    Eigen::MatrixXd selectorMatrix; /**< Selector matrix. */
    Eigen::MatrixXd Jc_invM; /**< Jc_invM = JacobianContacts / massMatrix. */
    Eigen::MatrixXd
        Lambda; /**< Lambda matrix, i.e. Lambda = Jc_invM * selectorMatrix */
    Eigen::MatrixXd
        dampedPinvLambda; /**< Damped Pseudo-inverse of the Lambda matrix. */
    Eigen::MatrixXd
        nullLambda;        /**< Null space projector of the Lambda matrix. */
    Eigen::MatrixXd Sigma; /**< Sigma matrix. */
    Eigen::VectorXd tauModel /**< joint torques computed from the model. */;
    Eigen::VectorXd
        u0; /**< joint feedback torques computed from the postural task. */
    Eigen::VectorXd contactWrenches; /**< Contact wrenches, arranged like
                                        [leftFootWrench; rightFootWrench] */
  };

  FullDynamics m_fullDynamics; /**< Full dynamics of the robot. */

  struct Kinematics {
    Eigen::MatrixXd
        jacobianContacts; /**< Stacked Jacobian of the contact frames. */
    Eigen::MatrixXd
        jacobianLeftContact; /**< Jacobian of the left contact frame. */
    Eigen::MatrixXd
        jacobianRightContact; /**< Jacobian of the right contact frame. */
    Eigen::VectorXd jacobianDotNuContacts;    /**< Stacked Bias accelerations of
                                                 the contact frames. */
    Eigen::VectorXd jacobianDotNuLeftContact; /**< Bias acceleration of the left
                                                 contact frame. */
    Eigen::VectorXd jacobianDotNuRightContact; /**< Bias acceleration of the
                                                  right contact frame. */
  };

  Kinematics m_kinematics; /**< Kinematics of the robot. */

  struct TorqueControlMode {
    bool isAskToSet{false}; /**< True if it has been ask to set */
    bool isSet{false};      /**< True if it has been set */
    std::future<bool>
        future; /**< Future object: True when control mode gets set */
    BipedalLocomotion::RobotInterface::IRobotControl::ControlMode value{
        BipedalLocomotion::RobotInterface::IRobotControl::ControlMode::Torque};
  };

  TorqueControlMode m_torqueControlMode; /**< Torque control mode. */

  std::chrono::nanoseconds
      m_computationTime; /**< Computation time of the control loop. */

  BipedalLocomotion::RobotInterface::PolyDriverDescriptor
      m_controlBoard; /**< Control board remapper. */

  BipedalLocomotion::RobotInterface::YarpRobotControl
      m_robotControl; /**< Robot control object.
                       */
  BipedalLocomotion::RobotInterface::YarpSensorBridge
      m_sensorBridge; /**< Sensor bridge object. */
  BipedalLocomotion::Estimators::LeggedOdometry
      m_floatingBaseEstimator; /**<Legged odometry object. */

  BipedalLocomotion::YarpUtilities::VectorsCollectionServer
      m_logDataServer; /**< Log data server. */

  MomentumBasedController::ReferenceGenerator
      m_referenceGenerator; /**< Reference generator. */

  struct ContactWrenchHandler {
    BipedalLocomotion::RobotInterface::PolyDriverDescriptor
        polyDriverDescriptor;
    BipedalLocomotion::Math::Wrenchd wrench;
  };

  std::unordered_map<std::string, ContactWrenchHandler>
      m_leftFootContacWrenches; /**< Contact wrenches of the left foot. */
  std::unordered_map<std::string, ContactWrenchHandler>
      m_rightFootContacWrenches; /**< Contact wrenches of the right foot. */
  std::unordered_map<std::string, ContactWrenchHandler>
      m_externalContactWrenches; /**< External contact wrenches. */

  /**
   * @brief Initialize the Yarp logger device.
   * @param handler parameters handler.
   * @return true if successfull.
   */
  bool initializeLogger(
      std::shared_ptr<
          const BipedalLocomotion::ParametersHandler::IParametersHandler>
          handler);

  /**
   * @brief log the data through the Yarp logger device.
   */
  void logData();

  /**
   * @brief Instantiate the sensor bridge.
   * @param handler parameters handler.
   * @return true if successfull.
   */
  bool instantiateSensorBridge(
      std::shared_ptr<
          const BipedalLocomotion::ParametersHandler::IParametersHandler>
          parametersHandler);

  /**
   * @brief Instantiate the legged odometry.
   * @param handler parameters handler.
   * @param modelPath path to the model.
   * @param jointLists list of controlled joints.
   * @param fixedJointsMap map of fixed joints.
   * @return true if successfull.
   */
  bool instantiateLeggedOdometry(
      std::shared_ptr<
          const BipedalLocomotion::ParametersHandler::IParametersHandler>
          handler,
      const std::string &modelPath, const std::vector<std::string> &jointLists,
      const std::unordered_map<std::string, double> &fixedJointsMap);

  /**
   * @brief Initialize the robot control.
   * @param parametersHandler parameters handler.
   * @return true if successfull.
   */
  bool initializeRobotControl(
      std::shared_ptr<
          const BipedalLocomotion::ParametersHandler::IParametersHandler>
          parametersHandler);

  /**
   * @brief Create the PolyDriver.
   * @param parametersHandler parameters handler.
   * @return true if successfull.
   */
  bool createPolyDriver(
      std::shared_ptr<
          const BipedalLocomotion::ParametersHandler::IParametersHandler>
          parametersHandler);

  /**
   * @brief Create the KinDyn object.
   * @param modelPath path to the model.
   * @param controlledJoints list of controlled joints.
   * @param fixedJointsMap map of fixed joints.
   * @return true if successfull.
   */
  bool
  createKinDyn(const std::string &modelPath,
               const std::vector<std::string> &controlledJoints,
               const std::unordered_map<std::string, double> &fixedJointsMap);

  /**
   * @brief Instantiate the QP solver.
   */
  void instantiateQPsolver();

  /**
   * @brief Compute the Anuglar Momentum Integral Error.
   * @param angularMomentumIntegralError angular momentum integral error.
   * @return true if successfull.
   */
  bool getAngularMomentumIntegralError(
      Eigen::Ref<Eigen::Vector3d> angularMomentumIntegralError);

  /**
   * @brief computes the constraint matrix and bias vector for applying friction
   * cones, unilateral constraints and local CoP constraints at contact
   * locations (assuming rigid contacts). The matrix C and the
   * bias vector b define the following linear inequality:
   *
   *                                    C*f < b
   *
   * f are the contact forces and moments w.r.t. a reference frames attached to
   * the contact location.
   * Each contact location will have its matrix C and bias vector b.
   */
  void computeRigidContactConstraints();

  /**
   * @brief Compute the number of rigid contact constraints.
   * @return the number of rigid contact constraints.
   */
  size_t getNumberOfRigidContactConstraints() const;

  /**
   * @brief Compute the Hessian and Gradient of the single support
   * configuration.
   * @param hessianMatrix hessian matrix.
   * @param gradient gradient vector.
   */
  void getOneFootHessianAndGradient(Eigen::Ref<Eigen::MatrixXd> hessianMatrix,
                                    Eigen::Ref<Eigen::VectorXd> gradient) const;

  /**
   * @brief Compute the Hessian and Gradient of the double support
   * configuration.
   * @param hessianMatrix hessian matrix.
   * @param gradient gradient vector.
   */
  void getTwoFeetHessianAndGradient(Eigen::Ref<Eigen::MatrixXd> hessianMatrix,
                                    Eigen::Ref<Eigen::VectorXd> gradient) const;

  /**
   * @brief Update the floating base estimator.
   * @return true if successful.
   */
  bool updateFloatingBase();

public:
  /**
   * @brief Initialize the Momentum Based Torque Controller.
   * @param parametersHandler parameters handler.
   * @return true if successful.
   */
  bool
  initialize(std::shared_ptr<
             const BipedalLocomotion::ParametersHandler::IParametersHandler>
                 parametersHandler);

  /**
   * @brief Advance the controller.
   * @return true if successful.
   */
  bool advance() final;

  /**
   * @brief Set the input of the controller.
   * @param input input of the controller. In our case it is an empty signal.
   * @return true if successful.
   */
  bool setInput(const Input &input) { return true; };

  WholeBodyController() = default;

  ~WholeBodyController();

  /**
   * @brief Stop the controller.
   */
  void stop();
};
} // namespace MomentumBasedController

#endif // MOMENTUM_BASED_TORQUE_CONTROL_CONTROLLER_H