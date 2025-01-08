/**
 * @file main.cpp
 * @authors Lorenzo Moretti, Ines Sorrentino
 * @copyright 2024 Istituto Italiano di Tecnologia (IIT). This software may be
 * modified and distributed under the terms of the BSD-3 clause license.
 */

#include <unordered_map>

#include <yarp/os/RFModule.h>

#include <iDynTree/EigenHelpers.h>
#include <iDynTree/Model.h>
#include <iDynTree/ModelLoader.h>

#include <BipedalLocomotion/Conversions/ManifConversions.h>
#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/RobotInterface/YarpHelper.h>
#include <BipedalLocomotion/RobotInterface/YarpRobotControl.h>
#include <BipedalLocomotion/RobotInterface/YarpSensorBridge.h>
#include <BipedalLocomotion/System/Clock.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

#include <MomentumBasedTorqueControl/Controller.h>

using namespace std::chrono_literals;

Eigen::Vector3d
radiansToDegrees(const Eigen::Ref<const Eigen::Vector3d> &radiansVector) {
  // Conversion factor from radians to degrees
  const double rad_to_deg = 180.0 / M_PI;

  // Create a vector to store the degrees
  Eigen::Vector3d degreesVector;

  // Convert each component from radians to degrees
  degreesVector = radiansVector * rad_to_deg;

  return degreesVector;
}

Eigen::MatrixXd pseudoInverse(const Eigen::Ref<const Eigen::MatrixXd> &mat,
                              double tolerance = 1e-6) {
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(mat, Eigen::ComputeThinU |
                                                 Eigen::ComputeThinV);
  Eigen::VectorXd singularValuesInv = svd.singularValues();

  // Invert all non-zero singular values (those greater than tolerance)
  for (int i = 0; i < singularValuesInv.size(); ++i) {
    if (singularValuesInv(i) > tolerance)
      singularValuesInv(i) = 1.0 / singularValuesInv(i);
    else
      singularValuesInv(i) = 0.0; // Zero out values below tolerance
  }

  // Pseudo-inverse calculation
  return svd.matrixV() * singularValuesInv.asDiagonal() *
         svd.matrixU().transpose();
}

Eigen::MatrixXd
matrixRightDivision(const Eigen::Ref<const Eigen::MatrixXd> &A,
                    const Eigen::Ref<const Eigen::MatrixXd> &B) {
  // Perform right division (solve A = X * B)
  // return A * B.completeOrthogonalDecomposition().pseudoInverse();
  return A * pseudoInverse(B);
}

Eigen::MatrixXd
positiveMatrixRightDivision(const Eigen::Ref<const Eigen::MatrixXd> &A,
                            const Eigen::Ref<const Eigen::MatrixXd> &B) {
  // Perform right division (solve A = X * B)
  // Assuming B is positive definite, use Cholesky decomposition to solve A / B
  // return A * B.llt().solve(Eigen::MatrixXd::Identity(B.rows(), B.cols()));
  return A * pseudoInverse(B);
}

Eigen::MatrixXd
pseudoInverseDamped(const Eigen::Ref<const Eigen::MatrixXd> &mat,
                    double regDamp = 0) {
  // mat' / (mat * mat' + regDamp * eye(mat.rows(), mat.rows()))
  auto A = mat.transpose();
  auto B = mat * mat.transpose() +
           regDamp * Eigen::MatrixXd::Identity(mat.rows(), mat.rows());
  return matrixRightDivision(A, B);
}

namespace MomentumBasedController {

bool WholeBodyController::initialize(
    std::shared_ptr<
        const BipedalLocomotion::ParametersHandler::IParametersHandler>
        parametersHandler) {
  constexpr auto logPrefix = "[WholeBodyController::initialize]";

  if (parametersHandler == nullptr) {
    BipedalLocomotion::log()->error("{} The parameter handler is not valid.",
                                    logPrefix);
    return false;
  }

  BipedalLocomotion::log()->info("{} Create the polydriver.", logPrefix);
  if (!this->createPolyDriver(parametersHandler)) {
    BipedalLocomotion::log()->error("{} Unable to create the polydriver.",
                                    logPrefix);
    return false;
  }

  // This sleep is required to let the robot interface to be ready
  BipedalLocomotion::clock().sleepFor(1s);

  auto ptrTmp = parametersHandler->getGroup("ROBOT_INTERFACE").lock();
  if (ptrTmp == nullptr) {
    BipedalLocomotion::log()->error(
        "{} Unable to find the group 'ROBOT_INTERFACE'.", logPrefix);
    return false;
  }

  if (!ptrTmp->getParameter("joints_list", m_jointsList)) {
    BipedalLocomotion::log()->error("{} Unable to find the joint list.",
                                    logPrefix);
    return false;
  }
  m_numberOfJoints = m_jointsList.size();

  m_jointsPosition.resize(m_numberOfJoints);
  m_jointsVelocity.resize(m_numberOfJoints);
  m_jointsDesiredTorque.resize(m_numberOfJoints);

  std::vector<std::string> fixedJointsList;
  if (!ptrTmp->getParameter("fixed_joint_list_names", fixedJointsList)) {
    BipedalLocomotion::log()->error("{} Unable to find the joint list.",
                                    logPrefix);
    return false;
  }

  // check that none of the fixed joints is in the controlled joints
  for (const auto &fixedJoint : fixedJointsList) {
    if (std::find(m_jointsList.begin(), m_jointsList.end(), fixedJoint) !=
        m_jointsList.end()) {
      // If found, throw an error
      BipedalLocomotion::log()->error(
          "{} Fixed joint {} is also in the controlled joints "
          "list.",
          logPrefix, fixedJoint);
      return false;
    }
  }

  Eigen::VectorXd fixedJointPositions;
  if (!ptrTmp->getParameter("fixed_joint_list_values", fixedJointPositions)) {
    BipedalLocomotion::log()->error(
        "{} Unable to find the initial joint position.", logPrefix);
    return false;
  }

  if (fixedJointPositions.size() != fixedJointsList.size()) {
    BipedalLocomotion::log()->error(
        "{} The size of the initial joint position is different "
        "from the size of the joint list.",
        logPrefix);
    return false;
  }

  // create the map of the fixed joints with the corresponding position
  std::unordered_map<std::string, double> fixedJointsMap;

  for (size_t i{}; i < fixedJointsList.size(); i++) {
    fixedJointsMap[fixedJointsList[i]] = fixedJointPositions[i] * M_PI / 180.0;
  }

  std::string modelPath =
      yarp::os::ResourceFinder::getResourceFinderSingleton().findFileByName(
          "model.urdf");
  if (modelPath.empty()) {
    BipedalLocomotion::log()->error("{} Unable to find the model file.",
                                    logPrefix);
    return false;
  }

  if (!this->createKinDyn(modelPath, m_jointsList, fixedJointsMap)) {
    BipedalLocomotion::log()->error("{} Unable to initialize the kinDyn.",
                                    logPrefix);
    return false;
  }

  BipedalLocomotion::log()->info("{} Create the robot control helper.",
                                 logPrefix);
  if (!this->initializeRobotControl(parametersHandler)) {
    BipedalLocomotion::log()->error(
        "{} Unable to initialize the robotControl interface.", logPrefix);
    return false;
  }

  BipedalLocomotion::log()->info("{} Create the sensor bridge.", logPrefix);
  if (!this->instantiateSensorBridge(parametersHandler)) {
    BipedalLocomotion::log()->error(
        "{} Unable to initialize the sensor bridge.", logPrefix);
    return false;
  }

  BipedalLocomotion::log()->info("{} Create the legged odometry.", logPrefix);
  if (!this->instantiateLeggedOdometry(parametersHandler, modelPath,
                                       m_jointsList, fixedJointsMap)) {
    BipedalLocomotion::log()->error(
        "{} Unable to initialize the legged odometry.", logPrefix);
    return false;
  }

  BipedalLocomotion::log()->info("{} Initialize the logger.", logPrefix);
  if (!this->initializeLogger(parametersHandler)) {
    BipedalLocomotion::log()->error("{} Unable to initialize the logger.",
                                    logPrefix);
    return false;
  }

  // intialize the gains and parameters of whole body control
  auto wbcParametersHandler =
      parametersHandler->getGroup("WHOLE_BODY_CONTROLLER").lock();

  if (wbcParametersHandler == nullptr) {
    BipedalLocomotion::log()->error(
        "{} Unable to find the group 'WHOLE_BODY_CONTROLLER'.", logPrefix);
    return false;
  }

  auto tmpHandler = wbcParametersHandler->getGroup("QP").lock();

  if (tmpHandler == nullptr) {
    BipedalLocomotion::log()->error(
        "{} Unable to find the group 'QP' in the whole body controller.",
        logPrefix);
    return false;
  }

  if (!tmpHandler->getParameter("enable_fallback_to_unconstrained",
                                m_settings.fallbackToUnconstrainedQP)) {
    BipedalLocomotion::log()->error(
        "{} Unable to get the enable_fallback_to_unconstrained parameter",
        logPrefix);
    return false;
  }

  if (!tmpHandler->getParameter("max_number_of_consecutive_failures",
                                m_settings.maxNumberConsecutiveQPFailures)) {
    BipedalLocomotion::log()->info(
        "{} Unable to get the max_number_of_consecutive_failures parameter. "
        "No check of the number of consecutive QP failures will be performed.",
        logPrefix);
  }

  tmpHandler = wbcParametersHandler->getGroup("SATURATION").lock();

  if (tmpHandler == nullptr) {
    BipedalLocomotion::log()->error("{} Unable to find the group 'SATURATION' "
                                    "in the whole body controller.",
                                    logPrefix);
    return false;
  }

  if (!tmpHandler->getParameter("enable_torque_saturation",
                                m_settings.saturateTorques)) {
    BipedalLocomotion::log()->error(
        "{} Unable to get the enable_torque_saturation parameter", logPrefix);
    return false;
  }

  if (!tmpHandler->getParameter("max_torque", m_settings.maxTorque)) {
    BipedalLocomotion::log()->error("{} Unable to get the max_torque parameter",
                                    logPrefix);
    return false;
  }

  tmpHandler = wbcParametersHandler->getGroup("GAINS").lock();

  if (tmpHandler == nullptr) {
    BipedalLocomotion::log()->error(
        "{} Unable to find the group 'GAINS' in the whole body controller.",
        logPrefix);
    return false;
  }

  if (!tmpHandler->getParameter("kp_angular_momentum",
                                m_gains.kpAngularMomentum)) {
    BipedalLocomotion::log()->error(
        "{} Unable to get the kp_angular_momentum parameter");
    return false;
  }

  if (!tmpHandler->getParameter("ki_angular_momentum",
                                m_gains.kiAngularMomentum)) {
    BipedalLocomotion::log()->error(
        "{} Unable to get the ki_angular_momentum parameter");
    return false;
  }

  if (!tmpHandler->getParameter("kp_com", m_gains.kpCoM)) {
    BipedalLocomotion::log()->error("{} Unable to get the kp_com parameter");
    return false;
  }

  if (!tmpHandler->getParameter("kd_com", m_gains.kdCoM)) {
    BipedalLocomotion::log()->error("{} Unable to get the kd_com parameter");
    return false;
  }

  if (!tmpHandler->getParameter("kp_postural", m_gains.kpPostural)) {
    BipedalLocomotion::log()->error(
        "{} Unable to get the kp_postural parameter");
    return false;
  }

  if (!tmpHandler->getParameter("kd_postural", m_gains.kdPostural)) {
    BipedalLocomotion::log()->error(
        "{} Unable to get the kd_postural parameter");
    return false;
  }

  if (!(m_gains.kpPostural.size() == m_numberOfJoints)) {
    BipedalLocomotion::log()->error(
        "{} Wrong size of kp_postural parameter. Given {}, expected {} ",
        logPrefix, m_gains.kpPostural.size(), m_numberOfJoints);
    return false;
  }

  if (!(m_gains.kdPostural.size() == m_numberOfJoints)) {
    BipedalLocomotion::log()->error(
        "{} Wrong size of kd_postural parameter. Given {}, expected {} ",
        logPrefix, m_gains.kdPostural.size(), m_numberOfJoints);
    return false;
  }

  tmpHandler = wbcParametersHandler->getGroup("REGULARIZERS").lock();

  if (tmpHandler == nullptr) {
    BipedalLocomotion::log()->error(
        "{} Unable to find the group 'REGULARIZERS' in the whole body "
        "controller.",
        logPrefix);
    return false;
  }

  if (!tmpHandler->getParameter("hessianQP", m_regularizers.hessianQP)) {
    BipedalLocomotion::log()->error("{} Unable to get the hessianQP parameter");
    return false;
  }

  if (!tmpHandler->getParameter("tolerance_pseudoinverse",
                                m_regularizers.tolerancePinv)) {
    BipedalLocomotion::log()->error(
        "{} Unable to get the tolerance_pseudoinverse parameter");
    return false;
  }

  if (!tmpHandler->getParameter("damping_factor_pseudoinverse_lambda",
                                m_regularizers.dampingFactorPinvLambda)) {
    BipedalLocomotion::log()->error(
        "{} Unable to get the damping_factor_pseudoinverse parameter");
    return false;
  }

  if (!tmpHandler->getParameter("damping_factor_pseudoinverse_basevel",
                                m_regularizers.dampingFactorPinvBaseVel)) {
    BipedalLocomotion::log()->error(
        "{} Unable to get the damping_factor_pseudoinverse parameter");
    return false;
  }

  tmpHandler = wbcParametersHandler->getGroup("RIGID_CONTACTS").lock();

  if (tmpHandler == nullptr) {
    BipedalLocomotion::log()->error(
        "{} Unable to find the group 'RIGID_CONTACTS' in the whole body "
        "controller.",
        logPrefix);
    return false;
  }

  if (!tmpHandler->getParameter(
          "static_friction_coefficient",
          m_rigidContactConstraints.staticFrictionCoefficient)) {
    BipedalLocomotion::log()->error(
        "{} Unable to get the static_friction_coefficient parameter");
    return false;
  }

  if (!tmpHandler->getParameter(
          "torsional_friction_coefficient",
          m_rigidContactConstraints.torsionalFrictionCoefficient)) {
    BipedalLocomotion::log()->error(
        "{} Unable to get the torsional_friction_coefficient parameter");
    return false;
  }

  if (!tmpHandler->getParameter("minimum_normal_force",
                                m_rigidContactConstraints.fZmin)) {
    BipedalLocomotion::log()->error(
        "{} Unable to get the minimum_normal_force parameter");
    return false;
  }

  int numberOfPoints;
  if (!tmpHandler->getParameter("number_of_points_per_quadrant",
                                numberOfPoints)) {
    BipedalLocomotion::log()->error(
        "{} Unable to get the number_of_points_per_quadrant parameter");
    return false;
  }
  m_rigidContactConstraints.numberOfPoints = numberOfPoints;

  Eigen::Vector4d contactAreaSize;
  if (!tmpHandler->getParameter("contact_area_size", contactAreaSize)) {
    BipedalLocomotion::log()->error(
        "{} Unable to get the contact_area_size parameter");
    return false;
  }
  m_rigidContactConstraints.contactAreaSize << contactAreaSize(0),
      contactAreaSize(1), contactAreaSize(2), contactAreaSize(3);

  // intialize the dynamics
  m_totalMass = m_kinDyn->model().getTotalMass();

  m_gravityWrench.resize(6);
  m_gravityWrench << 0.0, 0.0,
      -BipedalLocomotion::Math::StandardAccelerationOfGravitation * m_totalMass,
      0.0, 0.0, 0.0;

  // initialize the centroidal dynamics
  m_centroidalDynamics.A_left.setIdentity(6, 6);
  m_centroidalDynamics.A_right.setIdentity(6, 6);
  m_centroidalDynamics.A.setIdentity(6, 12);
  m_centroidalDynamics.pinvA.setIdentity(12, 6);
  m_centroidalDynamics.nullSpaceProjectorA.setIdentity(12, 12);
  m_centroidalDynamics.f_LDotStar.resize(12);
  m_centroidalDynamics.L_DotStar.resize(6);

  // initialize the full dynamics
  m_fullDynamics.massMatrix.resize(6 + m_numberOfJoints, 6 + m_numberOfJoints);
  m_fullDynamics.massMatrixB.resize(6, 6);
  m_fullDynamics.massMatrixS.resize(m_numberOfJoints, m_numberOfJoints);
  m_fullDynamics.massMatrixBS.resize(6, 6 + m_numberOfJoints);
  m_fullDynamics.massMatrixBar.resize(m_numberOfJoints, m_numberOfJoints);
  m_fullDynamics.h.resize(6 + m_numberOfJoints);
  m_fullDynamics.selectorMatrix.resize(6 + m_numberOfJoints, m_numberOfJoints);
  m_fullDynamics.Jc_invM.resize(12, 6 + m_numberOfJoints);
  m_fullDynamics.Lambda.resize(12, m_numberOfJoints);
  m_fullDynamics.dampedPinvLambda.resize(m_numberOfJoints, 12);
  m_fullDynamics.nullLambda.resize(m_numberOfJoints, m_numberOfJoints);
  m_fullDynamics.Sigma.resize(m_numberOfJoints, 12);
  m_fullDynamics.tauModel.resize(m_numberOfJoints);
  m_fullDynamics.u0.resize(m_numberOfJoints);
  m_fullDynamics.contactWrenches.resize(12);

  // initialize the kinematics
  m_kinematics.jacobianLeftContact.resize(6, 6 + m_numberOfJoints);
  m_kinematics.jacobianRightContact.resize(6, 6 + m_numberOfJoints);
  m_kinematics.jacobianContacts.resize(12, 6 + m_numberOfJoints);
  m_kinematics.jacobianDotNuLeftContact.resize(6);
  m_kinematics.jacobianDotNuRightContact.resize(6);
  m_kinematics.jacobianDotNuContacts.resize(12);

  // instantiate the QP
  m_numberOfConstraintsPerFoot = this->getNumberOfRigidContactConstraints();
  if (!m_QPproblem.instantiateSolver(wbcParametersHandler->getGroup("QP"),
                                     2 * m_numberOfVariablesPerFoot,
                                     2 * m_numberOfConstraintsPerFoot)) {
    BipedalLocomotion::log()->error("{} Unable to instantiate the QP solver.",
                                    logPrefix);
  }

  // rigid contact constraints
  m_rigidContactConstraints.constraintsMatrixLeftFoot.resize(
      m_numberOfConstraintsPerFoot, m_numberOfVariablesPerFoot);
  m_rigidContactConstraints.constraintsMatrixRightFoot.resize(
      m_numberOfConstraintsPerFoot, m_numberOfVariablesPerFoot);
  m_rigidContactConstraints.constraintsVectorLeftFoot.resize(
      m_numberOfConstraintsPerFoot);
  m_rigidContactConstraints.constraintsVectorRightFoot.resize(
      m_numberOfConstraintsPerFoot);

  // get measurement from the robot
  if (!m_sensorBridge.advance()) {
    BipedalLocomotion::log()->error("{} Unable to advance the sensor bridge.",
                                    logPrefix);
    return false;
  };
  m_sensorBridge.getJointPositions(m_jointsPosition);
  m_jointsVelocity.setZero();

  BipedalLocomotion::log()->debug("{} joints position: {}.", logPrefix,
                                  m_jointsPosition.transpose());

  // initialize the feet pose for the legged odometry
  m_worldLeftFootTransform.setIdentity();
  m_worldRightFootTransform.setIdentity();

  // update the floating base
  if (!this->updateFloatingBase()) {
    BipedalLocomotion::log()->error("{} Unable to update the floating base.",
                                    logPrefix);
    return false;
  }

  // update the robot state
  Eigen::Vector3d gravity = m_gravityWrench.head<3>() / m_totalMass;

  if (!m_kinDyn->setRobotState(
          m_baseTransform.transform(), m_jointsPosition,
          iDynTree::make_span(m_baseVelocity.data(), manif::SE3d::Tangent::DoF),
          m_jointsVelocity, gravity)) {
    BipedalLocomotion::log()->error("{} Unable to set the robot state.",
                                    logPrefix);
    return false;
  }

  // get the initial state
  m_worldLeftFootTransform = BipedalLocomotion::Conversions::toManifPose(
      m_kinDyn->getWorldTransform("l_sole"));
  m_worldRightFootTransform = BipedalLocomotion::Conversions::toManifPose(
      m_kinDyn->getWorldTransform("r_sole"));

  BipedalLocomotion::log()->debug("{} Left Sole initial position: {}",
                                  logPrefix,
                                  BipedalLocomotion::Conversions::toManifPose(
                                      m_kinDyn->getWorldTransform("l_sole"))
                                      .translation()
                                      .transpose());

  BipedalLocomotion::log()->debug("{} Right Sole initial position: {}",
                                  logPrefix,
                                  BipedalLocomotion::Conversions::toManifPose(
                                      m_kinDyn->getWorldTransform("r_sole"))
                                      .translation()
                                      .transpose());

  // initialize the reference generator
  m_referenceGenerator.initialize(
      parametersHandler->getGroup("REFERENCE_GENERATOR").lock());
  m_referenceGenerator.setInitialState(m_jointsPosition);

  return true;
}

bool WholeBodyController::advance() {
  constexpr auto logPrefix = "[WholeBodyController::advance]";

  auto startTime = std::chrono::steady_clock::now();

  Eigen::VectorXd QPconstraintsVector, QPgradient;
  Eigen::MatrixXd QPconstraintsMatrix, QPhessianMatrix;

  QPconstraintsMatrix.resize(2 * m_numberOfConstraintsPerFoot,
                             2 * m_numberOfVariablesPerFoot);
  QPconstraintsVector.resize(2 * m_numberOfConstraintsPerFoot);
  QPhessianMatrix.resize(2 * m_numberOfVariablesPerFoot,
                         2 * m_numberOfVariablesPerFoot);
  QPgradient.resize(2 * m_numberOfVariablesPerFoot);

  // get the Feedback from the robot
  if (!m_sensorBridge.advance()) {
    BipedalLocomotion::log()->error("{} Unable to get the robot state.",
                                    logPrefix);
    return false;
  }

  if (!m_sensorBridge.getJointPositions(m_jointsPosition) ||
      !m_sensorBridge.getJointVelocities(m_jointsVelocity)) {
    BipedalLocomotion::log()->error(
        "{} Unable to get the joint positions and velocities.", logPrefix);
    return false;
  }

  // update the Base Transform
  if (!this->updateFloatingBase()) {
    BipedalLocomotion::log()->error("{} Unable to update the floating base.",
                                    logPrefix);
    return false;
  };

  // update the kinDyn object
  Eigen::Vector3d gravity = m_gravityWrench.head<3>() / m_totalMass;
  if (!m_kinDyn->setRobotState(
          m_baseTransform.transform(), m_jointsPosition,
          iDynTree::make_span(m_baseVelocity.data(), manif::SE3d::Tangent::DoF),
          m_jointsVelocity, gravity)) {
    BipedalLocomotion::log()->error("{} Unable to set the robot state.",
                                    logPrefix);
    return false;
  }

  // update kinematics
  m_worldLeftFootTransform = BipedalLocomotion::Conversions::toManifPose(
      m_kinDyn->getWorldTransform("l_sole"));
  m_worldRightFootTransform = BipedalLocomotion::Conversions::toManifPose(
      m_kinDyn->getWorldTransform("r_sole"));
  m_comPosition = iDynTree::toEigen(m_kinDyn->getCenterOfMassPosition());
  m_comVelocity = iDynTree::toEigen(m_kinDyn->getCenterOfMassVelocity());

  if (m_isFirstStep) {
    m_comPositionInitial = m_comPosition;
    m_isFirstStep = false;
  }

  // get Adjoint Matrices
  m_centroidalDynamics.A_left.block<3, 3>(3, 0) =
      iDynTree::skew(m_worldLeftFootTransform.translation() - m_comPosition);
  m_centroidalDynamics.A_right.block<3, 3>(3, 0) =
      iDynTree::skew(m_worldRightFootTransform.translation() - m_comPosition);

  m_centroidalDynamics.A.block<6, 6>(0, 0) = m_centroidalDynamics.A_left;
  m_centroidalDynamics.A.block<6, 6>(0, 6) = m_centroidalDynamics.A_right;

  // The momentum rate of change equals the summation of the external forces and
  // moments, i.e.:
  //
  //     LDot = A*f + f_grav (1)
  //
  //  where A is the matrix mapping the forces and moments into the
  //  momentum equations, f_grav is the gravity force, f is a vector stacking
  //  all the external forces and moments acting on the robot as follows:
  //
  //     f = [f_left; f_right]
  //
  //  where f_left are the forces and moments acting on the left foot and
  //  f_right are the forces and moments acting on the right foot.

  // We would like to achieve a desired momentum's dynamics:
  //
  //    LDot_star = LDot_des - KP_momentum*(L-LDes) - KI_momentum*(intL-intLDes)
  //
  // where intL is the integral of the momentum. Assume the contact forces
  // and moments can be considered as control inputs of Eq. (1). Then, the
  // problem is to find f such that:
  //
  //    LDot_star = A*f + f_grav (2)
  //
  // We must now distinguish two different cases:
  //
  // CASE 1: the robot is balancing on one foot. In this case, the solution
  //         to Eq. (2) is:
  //
  //    f = A^(-1)*(LDot_star - f_grav) (3)
  //
  // CASE 2: the robot is balancing on two feet. In this case, there is
  //         redundancy as there are more control inputs (12) than variables
  //         to control (6). Therefore one can write:
  //
  //    f = pinvA*(LDot_star - f_grav) + Na*f_0 (4)
  //
  // where pinvA is the pseudoinverse of matrix A and Na is its null space
  // projector. f_0 is a free variable that does not affect the momentum
  // dynamics Eq (1).

  // Advance the reference generator
  if (!m_referenceGenerator.advance()) {
    BipedalLocomotion::log()->error(
        "{} Unable to advance the reference generator.", logPrefix);
    return false;
  }
  m_references = m_referenceGenerator.getOutput();

  // Define the desired CoM Dynamics (therefore Linear Momentum)
  Eigen::Vector3d comPositionError, comVelocityError, comAcceleration_star;

  comPositionError =
      (m_comPositionInitial + m_references.comDeltaPosition) - m_comPosition;

  comVelocityError = m_references.comVelocity - m_comVelocity;

  comAcceleration_star = m_references.comAcceleration +
                         m_gains.kpCoM.cwiseProduct(comPositionError) +
                         m_gains.kdCoM.cwiseProduct(comVelocityError);

  // Define the desired Centroidal Momentum dynamics
  Eigen::VectorXd centroidalMomentum =
      iDynTree::toEigen(m_kinDyn->getCentroidalTotalMomentum());

  Eigen::Vector3d angularMomentumIntegralError;

  if (!this->getAngularMomentumIntegralError(angularMomentumIntegralError)) {
    BipedalLocomotion::log()->error(
        "{} Unable to get the angular momentum integral error.", logPrefix);
    return false;
  }

  m_centroidalDynamics.L_DotStar.head<3>() = m_totalMass * comAcceleration_star;
  m_centroidalDynamics.L_DotStar.tail<3>() =
      -m_gains.kpAngularMomentum.cwiseProduct(centroidalMomentum.tail<3>()) -
      m_gains.kiAngularMomentum.cwiseProduct(angularMomentumIntegralError);

  //// CASE 1: one foot balancing
  //
  // In this case, we make use of a QP solver. In particular, Eq. (3) is
  // rewritten as:
  //
  //    f^T*A^T*A*f - f^T*A^T*(LDot_star - f_grav) = 0 (5)
  //
  // that is the quadratic problem associated with Eq. (3). Now rewrite
  // Eq. (5) as:
  //
  //    f^T*Hessian*f + f^T*gradient = 0
  //
  // where Hessian = A^T*A and gradient = - A^T*(LDot_star - f_grav). Now
  // it is possible to solve the folowing QP problem:
  //
  // f_star = argmin_f |f^T*Hessian*f + f^T*gradient|^2
  //
  //          s.t. C*f < b
  //
  // where the inequality constraints represent the unilateral, friction
  // cone and local CoP constraints at the foot.

  // Update constraint matrices. The contact wrench associated with the
  // left foot (resp. right foot) is subject to the following constraint:
  //
  //     ConstraintMatrixLeftFoot * l_sole_f_left < constraintsVector
  //
  // In this case, however, f_left is expressed w.r.t. the frame l_sole,
  // which is solidal to the left foot. The contact forces f used in the
  // controller however are expressed w.r.t. the frame l_sole[w], that is
  // a frame with the origin at the contact location but the orientation
  // of the inertial frame. For this reason, the mapping between the two
  // frames is given by:
  //
  //    l_sole_f_left = blkdiag(l_sole_R_w, l_sole_R_w) * l_sole[w]_f_left
  //
  // therefore we rewrite the contact constraints as:
  //
  //    ConstraintMatrixLeftFoot * blkdiag(l_sole_R_w, l_sole_R_w) *
  //    l_sole[w]_f_left < constraintsVector
  //
  // and this in the end results in updating the constraint matrix as follows:
  //
  //    ConstraintMatrixLeftFoot = ConstraintMatrixLeftFoot *
  //    blkdiag(l_sole_R_w, l_sole_R_w)
  //
  // The same holds for the right foot.

  Eigen::MatrixXd constraintsMatrix;
  Eigen::VectorXd constraintsVector;

  this->computeRigidContactConstraints();

  if (!m_references.isTwoFeetContactConfiguration) {

    // One foot constraints
    QPconstraintsVector.setZero();
    QPconstraintsMatrix.setZero();

    if (m_references.isLeftFootInContact) {

      QPconstraintsMatrix.block(0, 0, m_numberOfConstraintsPerFoot,
                                m_numberOfVariablesPerFoot) =
          m_rigidContactConstraints.constraintsMatrixLeftFoot;

      QPconstraintsVector.head(m_numberOfConstraintsPerFoot) =
          m_rigidContactConstraints.constraintsVectorLeftFoot;

    } else {

      QPconstraintsMatrix.block(
          m_numberOfConstraintsPerFoot, m_numberOfVariablesPerFoot,
          m_numberOfConstraintsPerFoot, m_numberOfVariablesPerFoot) =
          m_rigidContactConstraints.constraintsMatrixRightFoot;

      QPconstraintsVector.tail(m_numberOfConstraintsPerFoot) =
          m_rigidContactConstraints.constraintsVectorRightFoot;
    }

    // Hessian matrix and gradient QP one foot
    QPhessianMatrix.setIdentity();
    QPgradient.setZero();

    this->getOneFootHessianAndGradient(
        (m_references.isLeftFootInContact)
            ? QPhessianMatrix.block(0, 0, m_numberOfVariablesPerFoot,
                                    m_numberOfVariablesPerFoot)
            : QPhessianMatrix.block(
                  m_numberOfVariablesPerFoot, m_numberOfVariablesPerFoot,
                  m_numberOfVariablesPerFoot, m_numberOfVariablesPerFoot),
        (m_references.isLeftFootInContact)
            ? QPgradient.head(m_numberOfConstraintsPerFoot)
            : QPgradient.tail(m_numberOfConstraintsPerFoot));
  }

  //// CASE 2: two feet balancing
  //
  // In this case, we solve Eq (4) by means of the matrix pseudoinverse and
  // NOT through the QP. The QP is instead used to calculate the vector
  // projected in the null space (f_0). In particular, we choose f_0 in
  // order to minimize the joint torques magnitude. To do so, it is necessary
  // to write down the relationship between the joint torques and the
  // contact forces:
  //
  //    tau = pinvLambda*(Jc*invM*(h - Jc^T*f) -JcDot_nu) + NLambda*tau_0 (6)
  //
  // where tau_0 is given by the following equation:
  //
  //    tau_0 = hs - Msb*invMb*hb - (Js^T - Msb*invMb*Jb^T)*f + u_0
  //
  // where we have:
  //
  //    M = [Mb, Mbs;    h = [hb;    Jc = [Jb, Js]
  //         Msb, Ms];        hs];
  //
  // obtained by partitioning the dynamics in order to split the first
  // six rows and the remaining NDOF rows.
  //
  // u_0 instead are the feedback terms associated with the postural task,
  // and therefore are given by the following expression:
  //
  //    u_0 = -KP_postural*NLambda*Mbar*jointPosTilde
  //    -KD_postural*NLambda*Mbar*jointVel
  //
  // where Mbar =  Ms-Msb/Mb*Mbs.
  //
  // Now, let us rewrite Eq. (6) in order to isolate the terms which
  // depend on the contact forces:
  //
  //    tau = Sigma*f + tauModel  (7)
  //
  // where Sigma    = -(pinvLambda*Jc*invM*Jc^T + NLambda*(Js^T -
  // Msb*invMb*Jb^T))
  //
  //       tauModel = pinvLambda*(Jc*invM*h -JcDot_nu) + ...
  //                  NLambda*(hs - Msb*invMb*hb + u_0)
  //
  // Finally, we substitute Eq. (4) into Eq. (7) which gives:
  //
  //    tau = Sigma*pinvA*(LDot_star - f_grav) + Sigma*Na*f_0 + tauModel (8)
  //
  // minimizing the torques implies we would like to have tau = 0 in Eq.
  // (8) (note that it is not possible to achieve tau = 0 by choosing f_0)
  //
  // It is possible to write down Eq. (8) as a QP problem, as we
  // did for Eq. (5):
  //
  //    f_0^T*Hessian*f_0 + f_0^T*gradient = 0 (9)
  //
  // where Hessian  = transpose(Sigma*Na)*Sigma*Na
  //       gradient = transpose(Sigma*Na)*(Sigma*pinvA*(LDot_star - f_grav) +
  //       tauModel)
  //
  // The associated QP formulation is now:
  //
  // f_0_star = argmin_f_0 |f_0^T*Hessian*f_0 + f_0^T*gradient|^2
  //
  //            s.t. C*f_0 < b
  //
  // Note that in this way we are assuming that the part of the contact
  // forces dedicated to stabilize the momentum dynamics, i.e. the term
  //
  //    f_LDot = pinvA*(LDot_star - f_grav)
  //
  // does not violate the constraints.

  // Compute f_LDot
  m_centroidalDynamics.pinvA =
      pseudoInverse(m_centroidalDynamics.A, m_regularizers.tolerancePinv);
  m_centroidalDynamics.f_LDotStar =
      m_centroidalDynamics.pinvA *
      (m_centroidalDynamics.L_DotStar - m_gravityWrench);

  // compute the null space projector of A
  m_centroidalDynamics.nullSpaceProjectorA =
      (Eigen::MatrixXd::Identity(12, 12) -
       m_centroidalDynamics.pinvA * m_centroidalDynamics.A) *
      m_references.isLeftFootInContact * m_references.isRightFootInContact;

  // Compute the Sigma and tauModel
  //
  // NOTE that the formula Eq (7) will be used for computing the torques
  // also in case  the robot is balancing on ONE foot. In fact, in that
  // case, f will be a vector of the form (left foot balancing):
  //
  //    f = [f_left (from QP); zeros(6,1)];
  //
  // same holds for the right foot balancing. The additional zeros are
  // required in order to match the dimension of Sigma (NDOF x 12).

  // get contact Jacobians
  m_kinDyn->getFrameFreeFloatingJacobian("l_sole",
                                         m_kinematics.jacobianLeftContact);
  m_kinDyn->getFrameFreeFloatingJacobian("r_sole",
                                         m_kinematics.jacobianRightContact);
  m_kinematics.jacobianContacts.block(0, 0, 6, 6 + m_numberOfJoints) =
      m_kinematics.jacobianLeftContact * m_references.isLeftFootInContact;
  m_kinematics.jacobianContacts.block(6, 0, 6, 6 + m_numberOfJoints) =
      m_kinematics.jacobianRightContact * m_references.isRightFootInContact;

  // get the dot(Jc)*nu
  m_kinDyn->getFrameBiasAcc("l_sole", m_kinematics.jacobianDotNuLeftContact);
  m_kinDyn->getFrameBiasAcc("r_sole", m_kinematics.jacobianDotNuRightContact);

  m_kinematics.jacobianDotNuContacts.head<6>() =
      m_kinematics.jacobianDotNuLeftContact * m_references.isLeftFootInContact;
  m_kinematics.jacobianDotNuContacts.tail<6>() =
      m_kinematics.jacobianDotNuRightContact *
      m_references.isRightFootInContact;

  // get the Selector Matrix
  m_fullDynamics.selectorMatrix.setZero();
  m_fullDynamics.selectorMatrix.block(6, 0, m_numberOfJoints,
                                      m_numberOfJoints) =
      Eigen::MatrixXd::Identity(m_numberOfJoints, m_numberOfJoints);

  m_kinDyn->getFreeFloatingMassMatrix(m_fullDynamics.massMatrix);
  m_fullDynamics.massMatrixB = m_fullDynamics.massMatrix.block<6, 6>(0, 0);
  m_fullDynamics.massMatrixS =
      m_fullDynamics.massMatrix.block(6, 6, m_numberOfJoints, m_numberOfJoints);
  m_fullDynamics.massMatrixBS =
      m_fullDynamics.massMatrix.block(0, 6, 6, m_numberOfJoints);

  // get the matrix sigma
  m_fullDynamics.Jc_invM = positiveMatrixRightDivision(
      m_kinematics.jacobianContacts, m_fullDynamics.massMatrix);

  m_fullDynamics.Lambda =
      m_fullDynamics.Jc_invM * m_fullDynamics.selectorMatrix;

  m_fullDynamics.dampedPinvLambda = pseudoInverseDamped(
      m_fullDynamics.Lambda, m_regularizers.dampingFactorPinvLambda);

  m_fullDynamics.nullLambda =
      Eigen::MatrixXd::Identity(m_numberOfJoints, m_numberOfJoints) -
      m_fullDynamics.dampedPinvLambda * m_fullDynamics.Lambda;

  m_fullDynamics.Sigma =
      -(m_fullDynamics.dampedPinvLambda * m_fullDynamics.Jc_invM *
            m_kinematics.jacobianContacts.transpose() +
        m_fullDynamics.nullLambda *
            (m_kinematics.jacobianContacts
                 .block(0, 6, m_kinematics.jacobianContacts.rows(),
                        m_numberOfJoints)
                 .transpose()) -
        positiveMatrixRightDivision(m_fullDynamics.massMatrixBS.transpose(),
                                    m_fullDynamics.massMatrixB) *
            m_kinematics.jacobianContacts
                .block(0, 0, m_kinematics.jacobianContacts.rows(), 6)
                .transpose());

  // Mbar is the mass matrix associated with the joint dynamics, i.e.
  // Mbar = Ms - Mbs'/Mb*Mbs;
  // NullLambda_Mbar = NullLambda * Mbar;
  m_fullDynamics.massMatrixBar =
      m_fullDynamics.massMatrixS -
      positiveMatrixRightDivision(m_fullDynamics.massMatrixBS.transpose(),
                                  m_fullDynamics.massMatrixB) *
          m_fullDynamics.massMatrixBS;

  // get the joints Position Error
  Eigen::VectorXd jointPositionError;
  jointPositionError.resize(m_numberOfJoints);
  jointPositionError = m_references.jointsPosition - m_jointsPosition;

  // get gravity and coriolis (h)
  m_kinDyn->generalizedBiasForces(m_fullDynamics.h);

  // get the Tau Model and u0
  m_fullDynamics.u0 = -m_gains.kdPostural.cwiseProduct(m_jointsVelocity) +
                      m_gains.kpPostural.cwiseProduct(
                          m_references.jointsPosition - m_jointsPosition);

  m_fullDynamics.tauModel =
      m_fullDynamics.dampedPinvLambda *
          (m_fullDynamics.Jc_invM * m_fullDynamics.h -
           m_kinematics.jacobianDotNuContacts) +
      m_fullDynamics.nullLambda *
          (m_fullDynamics.h.tail(m_numberOfJoints) -
           positiveMatrixRightDivision(m_fullDynamics.massMatrixBS.transpose(),
                                       m_fullDynamics.massMatrixB) *
               m_fullDynamics.h.head<6>() +
           m_fullDynamics.u0);

  //// QP parameters for two feet standing
  //
  // When the robot stands on two feet, the control objective is
  // the minimization of the joint torques through the redundancy of the
  // contact forces. See Previous comments.

  // Get the inequality constraints matrices
  Eigen::MatrixXd ConstraintsMatrixBothFeet;
  Eigen::VectorXd bVectorConstraintsBothFeet;

  ConstraintsMatrixBothFeet.resize(2 * m_numberOfConstraintsPerFoot,
                                   2 * m_numberOfVariablesPerFoot);
  ConstraintsMatrixBothFeet.setZero();
  ConstraintsMatrixBothFeet.block(0, 0, m_numberOfConstraintsPerFoot,
                                  m_numberOfVariablesPerFoot) =
      m_rigidContactConstraints.constraintsMatrixLeftFoot;
  ConstraintsMatrixBothFeet.block(
      m_numberOfConstraintsPerFoot, m_numberOfVariablesPerFoot,
      m_numberOfConstraintsPerFoot, m_numberOfVariablesPerFoot) =
      m_rigidContactConstraints.constraintsMatrixRightFoot;

  bVectorConstraintsBothFeet.resize(2 * m_numberOfConstraintsPerFoot);
  bVectorConstraintsBothFeet
      << m_rigidContactConstraints.constraintsVectorLeftFoot,
      m_rigidContactConstraints.constraintsVectorRightFoot;

  // The optimization problem Eq. (9) seeks for the redundancy of the external
  // wrench that minimize joint torques. Recall that the contact wrench can
  // be written as:
  //
  //     f = f_LDot + Na*f_0
  //
  // Then, the constraints on the contact wrench is of the form
  //
  //     ConstraintsMatrixBothFeet*f < constraintsVector
  //
  // which in terms of f0 is:
  //
  // ConstraintsMatrixBothFeet*Na*f0 < constraintsVector -
  // ConstraintsMatrixBothFeet*f_LDot
  //
  // The constraints matrix and vector are then updated as follows:
  //
  //   ConstraintsMatrixTwoFeet = ConstraintsMatrixBothFeet*Na
  //   bVectorConstraintsTwoFeet = bVectorConstraintsBothFeet -
  //   ConstraintsMatrixBothFeet*f_LDot

  if (m_references.isTwoFeetContactConfiguration) {

    QPconstraintsMatrix =
        ConstraintsMatrixBothFeet * m_centroidalDynamics.nullSpaceProjectorA;

    QPconstraintsVector =
        bVectorConstraintsBothFeet -
        ConstraintsMatrixBothFeet * m_centroidalDynamics.f_LDotStar;

    // Evaluation of Hessian matrix and gradient vector for solving the
    // optimization problem Eq. (9)

    this->getTwoFeetHessianAndGradient(QPhessianMatrix, QPgradient);
  }

  // Update QP problem matrices
  m_QPproblem.setHessianMatrix(QPhessianMatrix);
  m_QPproblem.setGradient(QPgradient);
  m_QPproblem.setConstraints(QPconstraintsMatrix, QPconstraintsVector);

  // Solve the QP problem
  Eigen::VectorXd f0;
  f0.resize(2 * m_numberOfVariablesPerFoot);

  if (m_QPproblem.solve()) {

    // reset the number of consecutive fallbacks
    m_numberOfConsecutiveQPFailures = 0;

    // get optimal solution
    f0 = m_QPproblem.getSolution();

  } else {

    // QP has failed, fallback to the unconstrained solution (if enabled)
    if (m_settings.fallbackToUnconstrainedQP) {

      // Increase the max_number_of_consecutive_fallbacks
      m_numberOfConsecutiveQPFailures++;

      // Check if the maximum number of consecutive fallbacks has been reached
      // (if the check is enabled)
      if (m_settings.maxNumberConsecutiveQPFailures >= 0) {
        if (m_numberOfConsecutiveQPFailures >
            m_settings.maxNumberConsecutiveQPFailures) {
          BipedalLocomotion::log()->error(
              "{} Maximum number of consecutive fallbacks reached, stopping "
              "the controller.",
              logPrefix);
          return false;
        }
      }

      BipedalLocomotion::log()->warn(
          "{} Unable to solve the QP problem, fallback to the analytical "
          "unconstrained solution.",
          logPrefix);
      f0 = -pseudoInverse(QPhessianMatrix) * QPgradient;
    } else {
      BipedalLocomotion::log()->error("{} Unable to solve the QP problem.",
                                      logPrefix);
      return false;
    }
  }

  // Compute the contact wrenches
  if (m_references.isTwoFeetContactConfiguration) {

    m_fullDynamics.contactWrenches =
        m_centroidalDynamics.f_LDotStar +
        m_centroidalDynamics.nullSpaceProjectorA * f0;

  } else {

    m_fullDynamics.contactWrenches = f0;
    if (m_references.isLeftFootInContact) {
      m_fullDynamics.contactWrenches.tail<6>().setZero();
    } else {
      m_fullDynamics.contactWrenches.head<6>().setZero();
    }
  }

  // Compute the joint torques from eq (7)
  m_jointsDesiredTorque =
      m_fullDynamics.Sigma * m_fullDynamics.contactWrenches +
      m_fullDynamics.tauModel;

  // saturate the joint torques
  if (m_settings.saturateTorques) {
    m_jointsDesiredTorque =
        m_jointsDesiredTorque.cwiseMax(-m_settings.maxTorque)
            .cwiseMin(m_settings.maxTorque);
  }

  // set torque control mode
  if (!m_torqueControlMode.isAskToSet) {
    m_torqueControlMode.future =
        m_robotControl.setControlModeAsync(m_torqueControlMode.value);
    m_torqueControlMode.isAskToSet = true;
  }

  // check that control mode is set
  if (!m_torqueControlMode.isSet) {
    if (m_torqueControlMode.future.wait_for(std::chrono::seconds(0)) ==
        std::future_status::ready) {
      if (!m_torqueControlMode.isSet && m_torqueControlMode.future.get()) {
        BipedalLocomotion::log()->debug("{} Control mode set to torque",
                                        logPrefix);
        m_torqueControlMode.isSet = true;
      } else {
        BipedalLocomotion::log()->error("{} Unable to set the control mode",
                                        logPrefix);
        return false;
      }
    }
  }

  // Set the joint torques
  if (m_torqueControlMode.isSet) {
    if (!m_robotControl.setReferences(m_jointsDesiredTorque,
                                      m_torqueControlMode.value,
                                      m_jointsPosition)) {
      BipedalLocomotion::log()->error("{} Unable to set the reference",
                                      logPrefix);
      return false;
    }
  }

  // get computation time
  m_computationTime = std::chrono::steady_clock::now() - startTime;

  // log the data
  this->logData();

  return true;
}

bool WholeBodyController::getAngularMomentumIntegralError(
    Eigen::Ref<Eigen::Vector3d> angularMomentumIntegralError) {

  /**
  Given the assumption that the fixed foot is in contact with the ground and not
  moving (i.e. v_foot = 0 in R^6), the following equation holds:

    v_foot = J_foot * nu = Jb_foot * v_b + Js_foot * qdot = 0

  where Jb_foot and Js_foot are the portions of the Jacobian matrix J_foot
  related to the base and the joints, respectively. The above equation can be
  rewritten as:

    Js_foot * qdot = - Jb_foot * v_b

  And finally, we can retrieve base velocity v_b as:

    v_b = - Jb_foot^(-1) * Js_foot * qdot

  where Jb_foot^(-1) is the pseudoinverse of Jb_foot.
   */

  constexpr auto logPrefix =
      "[WholeBodyController::getAngularMomentumIntegralError]";

  if (angularMomentumIntegralError.size() != 3) {
    BipedalLocomotion::log()->error(
        "{} The angularMomentumIntegralError vector must have size 3.",
        logPrefix);
    return false;
  }

  Eigen::MatrixXd Jb_foot, Js_foot, J_foot;
  J_foot.resize(6, 6 + m_numberOfJoints);
  Jb_foot.resize(6, 6);
  Js_foot.resize(6, m_numberOfJoints);

  std::string fixedFootFrameName;
  if (m_references.useLeftAsFixedFoot) {
    fixedFootFrameName = "l_sole";
  } else {
    fixedFootFrameName = "r_sole";
  }

  m_kinDyn->getFrameFreeFloatingJacobian(fixedFootFrameName, J_foot);

  Jb_foot = J_foot.block(0, 0, 6, 6);
  Js_foot = J_foot.block(0, 6, 6, m_numberOfJoints);

  Eigen::VectorXd baseVelocityEquivalent;
  baseVelocityEquivalent.resize(6);
  Eigen::VectorXd jointsPositionError;
  jointsPositionError.resize(m_numberOfJoints);

  jointsPositionError = m_jointsPosition - m_references.jointsPosition;

  baseVelocityEquivalent =
      -pseudoInverseDamped(Jb_foot, m_regularizers.dampingFactorPinvBaseVel) *
      Js_foot * jointsPositionError;

  // update robot state to get the integral of the angular momentum

  Eigen::Vector3d gravity = m_gravityWrench.head<3>() / m_totalMass;
  manif::SE3d base_H_foot;

  if (!m_kinDyn->setJointPos(m_references.jointsPosition)) {
    BipedalLocomotion::log()->error(
        "{} Unable to set the joint position reference.", logPrefix);
    angularMomentumIntegralError.setZero();
    return false;
  }

  base_H_foot = BipedalLocomotion::Conversions::toManifPose(
      m_kinDyn->getRelativeTransform("root_link", fixedFootFrameName));

  if (!m_kinDyn->setRobotState(
          base_H_foot.inverse().transform(), m_references.jointsPosition,
          iDynTree::make_span(baseVelocityEquivalent.data(),
                              manif::SE3d::Tangent::DoF),
          jointsPositionError, gravity)) {

    BipedalLocomotion::log()->error("{} Unable to set the robot state.",
                                    logPrefix);

    angularMomentumIntegralError.setZero();

    return false;
  }

  angularMomentumIntegralError =
      iDynTree::toEigen(m_kinDyn->getCentroidalTotalMomentum()).tail<3>();

  // restore the robot state
  if (!m_kinDyn->setRobotState(
          m_baseTransform.transform(), m_jointsPosition,
          iDynTree::make_span(m_baseVelocity.data(), manif::SE3d::Tangent::DoF),
          m_jointsVelocity, gravity)) {
    BipedalLocomotion::log()->error("{} Unable to restore the robot state.",
                                    logPrefix);

    angularMomentumIntegralError.setZero();
    return false;
  }

  return true;
}

void WholeBodyController::computeRigidContactConstraints() {

  Eigen::MatrixXd constraintsMatrix; // ConstraintsMatrix per foot
  Eigen::VectorXd constraintsVector; // constraintsVector per foot

  // Compute friction cone constraints approximation with straight lines
  double segmentAngle =
      M_PI / 2.0 / (m_rigidContactConstraints.numberOfPoints - 1);
  Eigen::VectorXd angle = Eigen::VectorXd::LinSpaced(
      4 * (m_rigidContactConstraints.numberOfPoints - 2) + 4, 0,
      2 * M_PI - segmentAngle);

  Eigen::MatrixXd points(2, angle.size());
  points.row(0) = angle.array().cos();
  points.row(1) = angle.array().sin();
  size_t numberOfEquations = points.cols();
  assert(numberOfEquations ==
         (4 * (m_rigidContactConstraints.numberOfPoints - 2) + 4));

  // Define the inequality matrix A_ineq
  Eigen::MatrixXd A_ineq = Eigen::MatrixXd::Zero(numberOfEquations, 6);

  // Define equations for friction cone constraints
  double angularCoefficients{};
  double offsets{};
  int inequalityFactor{};

  for (size_t i = 0; i < numberOfEquations; ++i) {
    Eigen::Vector2d firstPoint = points.col(i);
    Eigen::Vector2d secondPoint = points.col((i + 1) % numberOfEquations);

    // Calculate the line passing through the above points
    angularCoefficients =
        (secondPoint.y() - firstPoint.y()) / (secondPoint.x() - firstPoint.x());
    offsets = firstPoint.y() - angularCoefficients * firstPoint.x();

    inequalityFactor = 1;

    // Adjust inequality factor if any angle is between pi and 2pi
    if (angle(i) > M_PI || angle((i + 1) % numberOfEquations) > M_PI) {
      inequalityFactor = -1;
    }

    // Populate A_ineq matrix for friction cone constraints
    A_ineq(i, 0) = -inequalityFactor * angularCoefficients;
    A_ineq(i, 1) = inequalityFactor;
    A_ineq(i, 2) = -inequalityFactor * offsets *
                   m_rigidContactConstraints.staticFrictionCoefficient;
  }

  // Define the constraints matrix for CoP and vertical force constraints
  // clang-format off
  Eigen::MatrixXd additionalConstraints(7, 6);
    additionalConstraints <<  0, 0,   -m_rigidContactConstraints.torsionalFrictionCoefficient,  0,  0,  1,
                              // torque_z - torsionalFrictionCoefficient * F_z < 0
                              0, 0,   -m_rigidContactConstraints.torsionalFrictionCoefficient,  0,  0, -1,  
                              // -torque_z - torsionalFrictionCoefficient * F_z < 0
                              0, 0,                                                        -1,  0,  0,  0,
                              // -F_z < -fZmin
                              0, 0,  m_rigidContactConstraints.contactAreaSize(0, 0),  0,  1,  0,
                              // torque_y + dimMinArea_x * F_z < 0
                              0, 0, -m_rigidContactConstraints.contactAreaSize(0, 1),  0, -1,  0,
                              // -torque_y - dimMaxArea_x * F_z < 0
                              0, 0,  m_rigidContactConstraints.contactAreaSize(1, 0), -1,  0,  0,
                              // -torque_x + dimMinArea_y * F_z < 0
                              0, 0, -m_rigidContactConstraints.contactAreaSize(1, 1),  1,  0,  0;
                              // torque_x - dimMaxArea_y * F_z < 0
  // clang-format on

  // Combine A_ineq and additionalConstraints into ConstraintsMatrix
  constraintsMatrix.resize(numberOfEquations + 7, 6);
  constraintsMatrix << A_ineq, additionalConstraints;

  // Define bias vector constraints (constraintsVector)
  constraintsVector.resize(numberOfEquations + 7, 1);
  constraintsVector.setZero();
  constraintsVector(numberOfEquations + 2) = -m_rigidContactConstraints.fZmin;

  // Left foot constraints and Right foot constraints
  Eigen::MatrixXd blkDiagMatrix;
  blkDiagMatrix.resize(6, 6);
  blkDiagMatrix.setZero();
  blkDiagMatrix.block<3, 3>(0, 0) =
      m_worldLeftFootTransform.rotation().transpose();
  blkDiagMatrix.block<3, 3>(3, 3) =
      m_worldLeftFootTransform.rotation().transpose();

  m_rigidContactConstraints.constraintsMatrixLeftFoot =
      constraintsMatrix * blkDiagMatrix;

  m_rigidContactConstraints.constraintsVectorLeftFoot = constraintsVector;

  blkDiagMatrix.block<3, 3>(0, 0) =
      m_worldRightFootTransform.rotation().transpose();
  blkDiagMatrix.block<3, 3>(3, 3) =
      m_worldRightFootTransform.rotation().transpose();

  m_rigidContactConstraints.constraintsMatrixRightFoot =
      constraintsMatrix * blkDiagMatrix;

  m_rigidContactConstraints.constraintsVectorRightFoot = constraintsVector;
}

size_t WholeBodyController::getNumberOfRigidContactConstraints() const {

  size_t constraintsNumber{};

  // Linear approximation of friction cone constraints
  constraintsNumber = 4 * (m_rigidContactConstraints.numberOfPoints - 2) + 4;

  // CoP and vertical force constraints
  constraintsNumber += 7;

  return constraintsNumber;
};

void WholeBodyController::getOneFootHessianAndGradient(
    Eigen::Ref<Eigen::MatrixXd> hessianMatrix,
    Eigen::Ref<Eigen::VectorXd> gradient) const {

  Eigen::MatrixXd A;
  A.resize(6, 6);
  A = m_centroidalDynamics.A_left * m_references.isLeftFootInContact *
          (1 - m_references.isRightFootInContact) +
      m_centroidalDynamics.A_right * m_references.isRightFootInContact *
          (1 - m_references.isLeftFootInContact);

  assert(hessianMatrix.size() == 6 * 6);
  hessianMatrix = A.transpose() * A +
                  Eigen::MatrixXd::Identity(6, 6) * m_regularizers.hessianQP;

  assert(gradient.size() == 6);
  gradient =
      -A.transpose() * (m_centroidalDynamics.L_DotStar - m_gravityWrench);
}

void WholeBodyController::getTwoFeetHessianAndGradient(
    Eigen::Ref<Eigen::MatrixXd> hessianMatrix,
    Eigen::Ref<Eigen::VectorXd> gradient) const {

  Eigen::MatrixXd SigmaNa;
  SigmaNa.resize(m_fullDynamics.Sigma.rows(),
                 m_centroidalDynamics.nullSpaceProjectorA.cols());
  SigmaNa = m_fullDynamics.Sigma * m_centroidalDynamics.nullSpaceProjectorA;

  assert(hessianMatrix.size() == SigmaNa.cols() * SigmaNa.cols());
  hessianMatrix = SigmaNa.transpose() * SigmaNa +
                  Eigen::MatrixXd::Identity(SigmaNa.cols(), SigmaNa.cols()) *
                      m_regularizers.hessianQP;

  assert(gradient.size() == SigmaNa.cols());
  gradient = SigmaNa.transpose() *
             (m_fullDynamics.tauModel +
              m_fullDynamics.Sigma * m_centroidalDynamics.f_LDotStar);
}

bool WholeBodyController::updateFloatingBase() {
  constexpr auto logPrefix = "[WholeBodyController::updateFloatingBase]";

  // set joints position and velocity measures
  if (!m_floatingBaseEstimator.setKinematics(m_jointsPosition,
                                             m_jointsVelocity)) {
    BipedalLocomotion::log()->error(
        "{} Unable to set the kinematics in the base estimator.", logPrefix);
    return false;
  }

  if (!m_floatingBaseEstimator.setContactStatus(
          "l_sole", m_references.isLeftFootInContact,
          std::chrono::nanoseconds(0))) {
    BipedalLocomotion::log()->info(
        "{} unable to set the contact status of the l_sole", logPrefix);
    return false;
  }
  if (!m_floatingBaseEstimator.setContactStatus(
          "r_sole", m_references.isRightFootInContact,
          std::chrono::nanoseconds(0))) {
    BipedalLocomotion::log()->info(
        "{} unable to set the contact status of the r_sole", logPrefix);
    return false;
  }

  // update the floating base estimator
  if (!m_floatingBaseEstimator.advance()) {
    BipedalLocomotion::log()->error(
        "{} Unable to update the floating base estimator.", logPrefix);
    return false;
  }

  auto newFixedFrameIndex = (m_references.useLeftAsFixedFoot)
                                ? m_kinDyn->model().getFrameIndex("l_sole")
                                : m_kinDyn->model().getFrameIndex("r_sole");

  manif::SE3d newFixedFramePose = (m_references.useLeftAsFixedFoot)
                                      ? m_worldLeftFootTransform
                                      : m_worldRightFootTransform;

  if (!m_floatingBaseEstimator.changeFixedFrame(
          newFixedFrameIndex, newFixedFramePose.quat(),
          newFixedFramePose.translation())) {
    BipedalLocomotion::log()->error(
        "{} Unable to change the fixed frame in the base "
        "estimator.",
        logPrefix);
    return false;
  }

  // get the base pose and velocity
  m_baseTransform = m_floatingBaseEstimator.getOutput().basePose;
  m_baseVelocity = m_floatingBaseEstimator.getOutput().baseTwist;

  return true;
}

bool WholeBodyController::createPolyDriver(
    std::shared_ptr<
        const BipedalLocomotion::ParametersHandler::IParametersHandler>
        parametersHandler) {
  constexpr auto logPrefix = "[WholeBodyController::createPolydriver]";

  std::string name;
  if (!parametersHandler->getParameter("name", name)) {
    BipedalLocomotion::log()->error("{} Unable to find the name.", logPrefix);
    return false;
  }

  auto ptr = parametersHandler->getGroup("ROBOT_INTERFACE").lock();
  if (ptr == nullptr) {
    BipedalLocomotion::log()->error("{} Robot interface options is empty.",
                                    logPrefix);
    return false;
  }

  auto tmp = ptr->clone();
  tmp->setParameter("local_prefix", name);

  m_controlBoard =
      BipedalLocomotion::RobotInterface::constructRemoteControlBoardRemapper(
          tmp);

  if (!m_controlBoard.isValid()) {
    BipedalLocomotion::log()->error(
        "{} the robot polydriver has not been constructed.", logPrefix);
    return false;
  }

  return true;
}

bool WholeBodyController::initializeLogger(
    std::shared_ptr<
        const BipedalLocomotion::ParametersHandler::IParametersHandler>
        handler) {

  constexpr auto logPrefix = "[WholeBodyController::initializeLogger]";

  if (!m_logDataServer.initialize(handler->getGroup("LOGGER"))) {
    BipedalLocomotion::log()->error(
        "{} Unable to initialize the Vector Collection Server.", logPrefix);
    return false;
  }

  m_logDataServer.populateMetadata(
      "flags", {"isLeftFootInContact", "isRightFootInContact",
                "isTwoFeetContactConfiguration", "useLeftAsFixedFoot",
                "isTorqueControlMode", "QPfailed"});

  m_logDataServer.populateMetadata("joint_state::position::measured",
                                   m_jointsList);
  m_logDataServer.populateMetadata("joint_state::velocity::measured",
                                   m_jointsList);
  m_logDataServer.populateMetadata("joint_state::position::desired",
                                   m_jointsList);
  m_logDataServer.populateMetadata("joint_state::torque::desired",
                                   m_jointsList);

  m_logDataServer.populateMetadata("com::position::measured", {"x", "y", "z"});
  m_logDataServer.populateMetadata("com::velocity::measured", {"x", "y", "z"});
  m_logDataServer.populateMetadata("com::position::desired", {"x", "y", "z"});
  m_logDataServer.populateMetadata("com::velocity::desired", {"x", "y", "z"});
  m_logDataServer.populateMetadata("com::acceleration::desired",
                                   {"x", "y", "z"});

  m_logDataServer.populateMetadata("base::position::measured", {"x", "y", "z"});
  m_logDataServer.populateMetadata("base::velocity::measured",
                                   {"x", "y", "z", "wx", "wy", "wz"});
  m_logDataServer.populateMetadata("base::orientation::measured::rpy",
                                   {"r", "p", "y"});
  m_logDataServer.populateMetadata("base::orientation::measured::quat",
                                   {"x", "y", "z", "w"});

  m_logDataServer.populateMetadata("contact_wrench::left_foot::desired",
                                   {"fx", "fy", "fz", "tx", "ty", "tz"});
  m_logDataServer.populateMetadata("contact_wrench::right_foot::desired",
                                   {"fx", "fy", "fz", "tx", "ty", "tz"});

  m_logDataServer.populateMetadata("computation_time::milliseconds",
                                   {"control_loop"});

  m_logDataServer.finalizeMetadata();

  return true;
}

void WholeBodyController::logData() {

  m_logDataServer.prepareData();
  m_logDataServer.clearData();

  std::vector<double> flags;
  flags.push_back((m_references.isLeftFootInContact) ? 1.0 : 0.0);
  flags.push_back((m_references.isRightFootInContact) ? 1.0 : 0.0);
  flags.push_back((m_references.isTwoFeetContactConfiguration) ? 1.0 : 0.0);
  flags.push_back((m_references.useLeftAsFixedFoot) ? 1.0 : 0.0);
  flags.push_back((m_torqueControlMode.isSet) ? 1.0 : 0.0);
  flags.push_back((!m_QPproblem.isSolved()) ? 1.0 : 0.0);

  m_logDataServer.populateData("flags", flags);

  m_logDataServer.populateData("joint_state::position::measured",
                               m_jointsPosition);
  m_logDataServer.populateData("joint_state::velocity::measured",
                               m_jointsVelocity);
  m_logDataServer.populateData("joint_state::position::desired",
                               m_references.jointsPosition);
  m_logDataServer.populateData("joint_state::torque::desired",
                               m_jointsDesiredTorque);

  m_logDataServer.populateData("com::position::measured", m_comPosition);
  m_logDataServer.populateData("com::velocity::measured", m_comVelocity);
  Eigen::Vector3d comPositionDesired =
      m_references.comDeltaPosition + m_comPositionInitial;
  m_logDataServer.populateData("com::position::desired", comPositionDesired);
  m_logDataServer.populateData("com::velocity::desired",
                               m_references.comVelocity);
  m_logDataServer.populateData("com::acceleration::desired",
                               m_references.comAcceleration);

  m_logDataServer.populateData("base::position::measured",
                               m_baseTransform.translation());
  m_logDataServer.populateData("base::velocity::measured",
                               m_baseVelocity.coeffs());
  m_logDataServer.populateData(
      "base::orientation::measured::rpy",
      radiansToDegrees(iDynTree::toEigen(
          BipedalLocomotion::Conversions::toiDynTreeRot(m_baseTransform.asSO3())
              .asRPY())));
  m_logDataServer.populateData("base::orientation::measured::quat",
                               m_baseTransform.quat().coeffs());

  m_logDataServer.populateData("contact_wrench::left_foot::desired",
                               m_fullDynamics.contactWrenches.head<6>());
  m_logDataServer.populateData("contact_wrench::right_foot::desired",
                               m_fullDynamics.contactWrenches.tail<6>());

  std::vector<double> computationTime{};
  computationTime.push_back(m_computationTime.count() / 1'000'000.0);

  m_logDataServer.populateData("computation_time::milliseconds",
                               computationTime);

  m_logDataServer.sendData();
}

bool WholeBodyController::instantiateSensorBridge(
    std::shared_ptr<
        const BipedalLocomotion::ParametersHandler::IParametersHandler>
        parametersHandler) {
  constexpr auto logPrefix = "[WholeBodyController::instantiateSensorBridge]";

  if (!m_sensorBridge.initialize(
          parametersHandler->getGroup("SENSOR_BRIDGE"))) {
    BipedalLocomotion::log()->error(
        "{} Unable to initialize the sensor bridge.", logPrefix);
    return false;
  }

  yarp::dev::PolyDriverList list;
  list.push(m_controlBoard.poly.get(), m_controlBoard.key.c_str());

  if (!m_sensorBridge.setDriversList(list)) {
    BipedalLocomotion::log()->error("{} Unable to set the driver list.",
                                    logPrefix);
    return false;
  }

  return true;
}

bool WholeBodyController::instantiateLeggedOdometry(
    std::shared_ptr<
        const BipedalLocomotion::ParametersHandler::IParametersHandler>
        handler,
    const std::string &modelPath, const std::vector<std::string> &jointLists,
    const std::unordered_map<std::string, double> &fixedJointsMap) {
  constexpr auto logPrefix = "[WholeBodyController::instantiateLeggedOdometry]";

  iDynTree::ModelLoader ml;
  if (!ml.loadReducedModelFromFile(modelPath, jointLists, fixedJointsMap)) {
    BipedalLocomotion::log()->error(
        "{} Unable to load the reduced model located in: {}.", logPrefix,
        modelPath);
    return false;
  }

  auto tmpKinDyn = std::make_shared<iDynTree::KinDynComputations>();
  if (!tmpKinDyn->loadRobotModel(ml.model())) {
    BipedalLocomotion::log()->error(
        "{} Unable to load a KinDynComputation object", logPrefix);
    return false;
  }

  if (handler->getGroup("FLOATING_BASE_ESTIMATOR").lock() == nullptr) {
    BipedalLocomotion::log()->error(
        "{} Unable to find the floating base estimator options.", logPrefix);
    return false;
  }

  if (!m_floatingBaseEstimator.initialize(
          handler->getGroup("FLOATING_BASE_ESTIMATOR"), tmpKinDyn)) {
    BipedalLocomotion::log()->error(
        "{} Unable to initialize the legged odometry.", logPrefix);
    return false;
  }

  return true;
}

bool WholeBodyController::initializeRobotControl(
    std::shared_ptr<
        const BipedalLocomotion::ParametersHandler::IParametersHandler>
        parametersHandler) {

  constexpr auto logPrefix = "[WholeBodyController::initializeRobotControl]";

  if (!m_robotControl.initialize(
          parametersHandler->getGroup("ROBOT_CONTROL"))) {
    BipedalLocomotion::log()->error(
        "{} Unable to initialize the robot control.", logPrefix);
    return false;
  }
  if (!m_robotControl.setDriver(m_controlBoard.poly)) {
    BipedalLocomotion::log()->error("{} Unable to set the polydriver.",
                                    logPrefix);
    return false;
  }

  return true;
}

bool WholeBodyController::createKinDyn(
    const std::string &modelPath, const std::vector<std::string> &jointLists,
    const std::unordered_map<std::string, double> &jointsToRemoveMap) {
  constexpr auto logPrefix = "[WholeBodyQPBlock::createKinDyn]";

  iDynTree::ModelLoader ml;
  if (!ml.loadReducedModelFromFile(modelPath, jointLists, jointsToRemoveMap)) {
    BipedalLocomotion::log()->error(
        "{} Unable to load the reduced model located in: {}.", logPrefix,
        modelPath);
    return false;
  }

  m_kinDyn = std::make_shared<iDynTree::KinDynComputations>();

  if (!m_kinDyn->loadRobotModel(ml.model())) {
    BipedalLocomotion::log()->error(
        "{} Unable to load a KinDynComputation object", logPrefix);
    return false;
  }
  BipedalLocomotion::log()->info("{} ndof: {}.", logPrefix,
                                 m_kinDyn->model().getNrOfDOFs());
  return true;
}

void WholeBodyController::stop() {
  m_robotControl.setControlMode(
      BipedalLocomotion::RobotInterface::IRobotControl::ControlMode::Position);

  BipedalLocomotion::log()->debug(
      "[WholeBodyController::stop] Control mode set to "
      "IRobotControl::ControlMode::Position");

  m_sensorBridge.advance();
  m_sensorBridge.getJointPositions(m_jointsPosition);

  m_robotControl.setReferences(
      m_jointsPosition,
      BipedalLocomotion::RobotInterface::IRobotControl::ControlMode::Position);
}

WholeBodyController::~WholeBodyController() { stop(); }

} // namespace MomentumBasedController