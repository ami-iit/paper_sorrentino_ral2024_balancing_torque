/**
 * @file main.cpp
 * @authors Lorenzo Moretti, Ines Sorrentino
 * @copyright 2024 Istituto Italiano di Tecnologia (IIT). This software may be
 * modified and distributed under the terms of the BSD-3 clause license.
 */

#ifndef REFERENCEGENERATOR_H
#define REFERENCEGENERATOR_H

#include <BipedalLocomotion/System/Source.h>

namespace MomentumBasedController {

struct WholeBodyControllerReference {
  Eigen::Vector3d comDeltaPosition; /**< Desired CoM position as a Delta wrt
                                           to the initial CoM position. */
  Eigen::Vector3d comVelocity;      /**< Desired CoM velocity */
  Eigen::Vector3d comAcceleration;  /**< Desired CoM acceleration */
  Eigen::VectorXd jointsPosition;   /**< Desired joint position */
  int isLeftFootInContact{1};       /**< 0: no contact, 1: contact */
  int isRightFootInContact{1};      /**< 0: no contact, 1: contact */
  bool useLeftAsFixedFoot{
      true}; /**< false: right foot is used as the fixed one,
                  true: left foot is used as the fixed one */
  bool isTwoFeetContactConfiguration{
      false};          /**< false: one foot contact, true: two feet contact */
  bool isValid{false}; /**< true if the reference is valid */
};

class ReferenceGenerator
    : public BipedalLocomotion::System::Source<WholeBodyControllerReference> {

  WholeBodyControllerReference
      m_output; /**< output of the reference generator */

  bool m_isInitialized{false};     /**< true if the reference generator is
                                      initialized */
  bool m_isInitialStateSet{false}; /**< true if the initial state is set */

  double m_time{0.0};      /**< time of the reference generator [s]*/
  double m_timeStep{0.01}; /**< time step of the reference generator [s]*/
  double m_initTime{0.0};  /**< start time of the trajectory. [s]*/
  double m_amplitude{0.1}; /**< frequency of the sinusoid [Hz]*/
  double m_frequency{1.0}; /**< amplitude of the sinusoid */
  Eigen::VectorXd m_initialJointsPosition; /**< initial joint position */

  /**
   * @brief Generate the trajectory for the CoM. When the time is less than the
   * initTime the trajectory is zero.
   * @param time current time.
   * @param amplitude amplitude of the sinusoid [m].
   * @param frequency frequency of the sinusoid [Hz].
   * @param position desired CoM position [m].
   * @param velocity desired CoM velocity [m/s].
   * @param acceleration desired CoM acceleration [m/(s^2)].
   * @param initTime start time of the trajectory [s].
   */
  void getCenterOfMassTrajectory(double time, double amplitude,
                                 double frequency, Eigen::Vector3d &position,
                                 Eigen::Vector3d &velocity,
                                 Eigen::Vector3d &acceleration,
                                 double initTime = 0.0);

  /**
   * @brief Generate the feet configuration.
   * @param time current time [s].
   * @param isLeftFootInContact 0: no contact, 1: contact.
   * @param isRightFootInContact 0: no contact, 1: contact.
   * @param useLeftAsFixedFoot false: right foot is used as the fixed one,
   * true: left foot is used as the fixed one.
   * @param isTwoFeetContactConfiguration false: one foot contact, true: two
   * feet contact.
   */
  void getFeetConfiguration(double time, int &isLeftFootInContact,
                            int &isRightFootInContact, bool &useLeftAsFixedFoot,
                            bool &isTwoFeetContactConfiguration);

public:
  ReferenceGenerator() = default;

  ~ReferenceGenerator() = default;

  /**
   * @brief Initialize the reference generator.
   * @param parametersHandler parameters handler.
   * @return true if successful.
   */
  bool
  initialize(std::shared_ptr<
             const BipedalLocomotion::ParametersHandler::IParametersHandler>
                 parametersHandler);

  /**
   * @brief Set the initial state of the reference generator.
   * @param initialJointsPosition initial joint position.
   */
  void setInitialState(const Eigen::VectorXd &initialJointsPosition);

  /**
   * @brief Advance the reference generator.
   * @return true if successful.
   */
  bool advance() final;

  /**
   * @brief Get the output of the reference generator.
   * @return output of the reference generator.
   */
  const WholeBodyControllerReference &getOutput() const final;

  /**
   * @brief Check if the output is valid.
   * @return true if the output is valid.
   */
  bool isOutputValid() const final;
};

} // namespace MomentumBasedController

#endif // REFERENCEGENERATOR_H