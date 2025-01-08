/**
 * @file main.cpp
 * @authors Lorenzo Moretti, Ines Sorrentino
 * @copyright 2024 Istituto Italiano di Tecnologia (IIT). This software may be
 * modified and distributed under the terms of the BSD-3 clause license.
 */

#include <BipedalLocomotion/TextLogging/Logger.h>

#include <MomentumBasedTorqueControl/ReferenceGenerator.h>

void deg2rad(Eigen::Ref<Eigen::VectorXd> v) {
  for (int i = 0; i < v.size(); i++) {
    v(i) = v(i) * M_PI / 180;
  }
}

namespace MomentumBasedController {

bool ReferenceGenerator::initialize(
    std::shared_ptr<
        const BipedalLocomotion::ParametersHandler::IParametersHandler>
        parametersHandler) {

  constexpr auto logPrefix = "[ReferenceGenerator::initialize]";

  if (!parametersHandler->getParameter("time_step", m_timeStep)) {
    BipedalLocomotion::log()->error(
        "{} Unable to find the time_step parameter.", logPrefix);
    return false;
  }

  if (!parametersHandler->getParameter("init_time", m_initTime)) {
    BipedalLocomotion::log()->error(
        "{} Unable to find the init_time parameter.", logPrefix);
    return false;
  }

  if (!parametersHandler->getParameter("amplitude", m_amplitude)) {
    BipedalLocomotion::log()->error(
        "{} Unable to find the amplitude parameter.", logPrefix);
    return false;
  }

  if (!parametersHandler->getParameter("frequency", m_frequency)) {
    BipedalLocomotion::log()->error(
        "{} Unable to find the frequency parameter.", logPrefix);
    return false;
  }

  m_isInitialized = true;

  return true;
}

void ReferenceGenerator::setInitialState(
    const Eigen::VectorXd &initialJointsPosition) {

  m_initialJointsPosition = initialJointsPosition;
  m_isInitialStateSet = true;
};

bool ReferenceGenerator::advance() {

  constexpr auto logPrefix = "[ReferenceGenerator::advance]";

  if (!m_isInitialized) {
    BipedalLocomotion::log()->error(
        "{} Reference generator is not initialized.", logPrefix);
    return false;
  }

  if (!m_isInitialStateSet) {
    BipedalLocomotion::log()->error("{} Initial state is not set.", logPrefix);
    return false;
  }

  // Generate trajectory for the CoM
  this->getCenterOfMassTrajectory(
      m_time, m_amplitude, m_frequency, m_output.comDeltaPosition,
      m_output.comVelocity, m_output.comAcceleration, m_initTime);

  // Generate trajectory for the joints
  m_output.jointsPosition = m_initialJointsPosition;

  // Generate the feet configuration
  this->getFeetConfiguration(
      m_time, m_output.isLeftFootInContact, m_output.isRightFootInContact,
      m_output.useLeftAsFixedFoot, m_output.isTwoFeetContactConfiguration);

  // advance the time
  m_time += m_timeStep;

  m_output.isValid = true;

  return true;
}

const WholeBodyControllerReference &ReferenceGenerator::getOutput() const {
  return m_output;
}

bool ReferenceGenerator::isOutputValid() const { return m_output.isValid; }

void ReferenceGenerator::getCenterOfMassTrajectory(
    double time, double amplitude, double frequency, Eigen::Vector3d &position,
    Eigen::Vector3d &velocity, Eigen::Vector3d &acceleration, double initTime) {

  position.setZero();
  velocity.setZero();
  acceleration.setZero();

  // sinusoidal trajectory just for the y component
  position.y() =
      (time >= initTime)
          ? amplitude * std::sin(2 * M_PI * frequency * (time - initTime))
          : 0.0;
  velocity.y() = (time >= initTime)
                     ? amplitude * 2 * M_PI * frequency *
                           std::cos(2 * M_PI * frequency * (time - initTime))
                     : 0.0;
  acceleration.y() =
      (time >= initTime)
          ? -amplitude * 4 * M_PI * M_PI * frequency * frequency *
                std::sin(2 * M_PI * frequency * (time - initTime))
          : 0.0;
}

void ReferenceGenerator::getFeetConfiguration(
    double time, int &isLeftFootInContact, int &isRightFootInContact,
    bool &useLeftAsFixedFoot, bool &isTwoFeetContactConfiguration) {

  isLeftFootInContact = 1;
  isRightFootInContact = 1;
  useLeftAsFixedFoot = true;
  isTwoFeetContactConfiguration = true;
}

} // namespace MomentumBasedController