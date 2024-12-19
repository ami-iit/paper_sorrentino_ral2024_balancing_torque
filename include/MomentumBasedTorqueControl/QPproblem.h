/**
 * @file main.cpp
 * @authors Lorenzo Moretti, Ines Sorrentino
 * @copyright 2024 Istituto Italiano di Tecnologia (IIT). This software may be
 * modified and distributed under the terms of the BSD-3 clause license.
 */

#ifndef QPSOLVER_H
#define QPSOLVER_H

#include <cstddef>
#include <memory>

#include <QpSolversEigen/QpSolversEigen.hpp>
#include <QpSolversEigen/Solver.hpp>

#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>

namespace MomentumBasedController {

// clang-format off
/**
 * @brief Quadratic programming problem solver. It solves the optimization problem 
 * in the form of:
 *
 *   x = argmmin 0.5 * x^T * H * x + g^T * x
 *   
 *       s.t. lb <= A * x <= ub
 *
 *   where: 
 *     - H is the Hessian matrix,
 *     - g is the gradient vector,
 *     - A is the linear constraints matrix,
 *     - lb and ub are the lower and upper bounds. 
 *     - x is the optimization variable.
 *
 * In our case, the ub vector is called linearConstraintsVector. 
 * While the lb vector is a dummy vector set to -INFTY. 
 */
// clang-format on

class QPproblem {

public:
  QPproblem();
  ~QPproblem();

  /**
   * @brief Instantiate the solver.
   * @param parametersHandler parameters handler.
   * @param numberOfVariables number of variables.
   * @param numberOfConstraints number of constraints.
   * @return true if successful.
   */
  bool instantiateSolver(
      std::weak_ptr<
          const BipedalLocomotion::ParametersHandler::IParametersHandler>
          parametersHandler,
      size_t numberOfVariables, size_t numberOfConstraints);

  /**
   * @brief Set the Hessian matrix.
   * @param Hessian Hessian matrix.
   * @return true if successful.
   */
  bool setHessianMatrix(const Eigen::Ref<const Eigen::MatrixXd> &Hessian);

  /**
   * @brief Set the gradient.
   * @param Gradient gradient vector.
   * @return true if successful.
   */
  bool setGradient(const Eigen::Ref<const Eigen::VectorXd> &Gradient);

  /**
   * @brief Set the linear constraints.
   * @param linearConstraintsMatrix linear constraints matrix.
   * @param linearConstraintsVector linear constraints vector.
   * @return true if successful.
   */
  bool setConstraints(
      const Eigen::Ref<const Eigen::MatrixXd> &linearConstraintsMatrix,
      const Eigen::Ref<const Eigen::VectorXd> &linearConstraintsVector);

  /**
   * @brief Solve the optimization problem.
   * @return true if successful.
   */
  bool solve();

  /**
   * @brief Check if the problem is solved.
   * @return true if the problem is solved.
   */
  bool isSolved() { return m_isSolved; }

  /**
   * @brief Get the solution of the optimization problem.
   * @return solution of the optimization problem.
   */
  Eigen::VectorXd getSolution();

private:
  std::unique_ptr<QpSolversEigen::Solver> m_solver; /**< Solver object. */
  size_t m_numberOfVariables;      /**< Number of optimization variables. */
  size_t m_numberOfConstraints;    /**< Number of constraints. */
  Eigen::MatrixXd m_HessianMatrix; /**< Hessian matrix. */
  Eigen::VectorXd m_Gradient;      /**< Gradient vector. */
  Eigen::MatrixXd m_linearConstraintsMatrix; /**< Linear constraints matrix. */
  Eigen::VectorXd m_linearConstraintsVector; /**< Linear constraints vector. */
  Eigen::VectorXd m_dummyLowerBoundVector;   /**< Dummy lower bound vector*/
  bool m_isSolved{false}; /**< True if the problem is solved. */

  /**
   * @brief Instantiate the OSQP solver.
   * @param parametersHandler parameters handler.
   * @return true if successful.
   */
  bool instantiateOsqpSolver(
      std::shared_ptr<
          const BipedalLocomotion::ParametersHandler::IParametersHandler>
          parametersHandler);

  /**
   * @brief Instantiate the PROXQP solver.
   * @param parametersHandler parameters handler.
   * @return true if successful.
   */
  bool instantiateProxqpSolver(
      std::shared_ptr<
          const BipedalLocomotion::ParametersHandler::IParametersHandler>
          parametersHandler);

  /**
   * @brief Initialize the solver.
   * @return true if successful.
   */
  bool initializeSolver();

  /**
   * @brief Update the lower bound of the solver.
   * @param lowerBound lower bound vector.
   */
  void updateProblem();
};
} // namespace MomentumBasedController

#endif // QPSOLVER_H