/**
 * @file main.cpp
 * @authors Lorenzo Moretti, Ines Sorrentino
 * @copyright 2024 Istituto Italiano di Tecnologia (IIT). This software may be
 * modified and distributed under the terms of the BSD-3 clause license.
 */

#include <BipedalLocomotion/TextLogging/Logger.h>

#include <QpSolversEigen/Constants.hpp>
#include <QpSolversEigen/Solver.hpp>

#include <MomentumBasedTorqueControl/QPproblem.h>

namespace MomentumBasedController {

QPproblem::QPproblem(){};
QPproblem::~QPproblem(){};

bool QPproblem::instantiateSolver(
    std::weak_ptr<
        const BipedalLocomotion::ParametersHandler::IParametersHandler>
        parametersHandler,
    size_t numberOfVariables, size_t numberOfConstraints) {

  constexpr auto logPrefix = "[QPproblem::instantiateSolver]";

  auto handler = parametersHandler.lock();

  if (handler == nullptr) {
    BipedalLocomotion::log()->error("{} The parameter handler is not valid.",
                                    logPrefix);
    return false;
  }

  // Get the solver name
  std::string solverName{"osqp"};
  if (!handler->getParameter("solver_name", solverName)) {
    BipedalLocomotion::log()->error(
        "{} Unable to find the solver_name parameter.", logPrefix);
    return false;
  }

  // Create the solver
  m_solver = std::make_unique<QpSolversEigen::Solver>();

  if (solverName == "osqp") {
    if (!this->instantiateOsqpSolver(handler)) {
      BipedalLocomotion::log()->error(
          "{} Unable to instantiate the 'osqp' solver.", logPrefix);
      return false;
    }
  } else if (solverName == "proxqp") {
    if (!this->instantiateProxqpSolver(handler)) {
      BipedalLocomotion::log()->error(
          "{} Unable to instantiate the 'proxqp' solver.", logPrefix);
      return false;
    }
  } else {
    BipedalLocomotion::log()->error("{} The solver '{}' is not supported.",
                                    logPrefix, solverName);
    return false;
  }

  // Set the number of variables and constraints
  m_numberOfVariables = numberOfVariables;
  m_numberOfConstraints = numberOfConstraints;
  m_solver->data()->setNumberOfVariables(m_numberOfVariables);
  m_solver->data()->setNumberOfConstraints(m_numberOfConstraints);

  // Initialize the matrices
  m_HessianMatrix.resize(m_numberOfVariables, m_numberOfVariables);
  m_HessianMatrix.setIdentity();

  m_Gradient.resize(m_numberOfVariables);
  m_Gradient.setZero();

  m_linearConstraintsMatrix.resize(m_numberOfConstraints, m_numberOfVariables);
  m_linearConstraintsMatrix.setZero();

  m_linearConstraintsVector.resize(m_numberOfConstraints);
  m_linearConstraintsVector.setZero();

  return true;
};

bool QPproblem::instantiateOsqpSolver(
    std::shared_ptr<
        const BipedalLocomotion::ParametersHandler::IParametersHandler>
        parametersHandler) {

  constexpr auto logPrefix = "[QPproblem::instantiateOsqpSolver]";

  this->m_solver->instantiateSolver("osqp");

  bool verbosity{false};
  if (!parametersHandler->getParameter("verbosity", verbosity)) {
    BipedalLocomotion::log()->error(
        "{} Unable to find the verbosity parameter.", logPrefix);
    return false;
  }

  if (!m_solver->setBooleanParameter("verbose", verbosity)) {
    BipedalLocomotion::log()->error("{} Unable to set the verbosity parameter.",
                                    logPrefix);
    return false;
  }

  int maxIterations{1000};
  if (!parametersHandler->getParameter("max_iterations", maxIterations)) {
    BipedalLocomotion::log()->error(
        "{} Unable to find the max_iterations parameter.", logPrefix);
    return false;
  }
  if (!m_solver->setIntegerParameter("max_iter", maxIterations)) {
    BipedalLocomotion::log()->error("{} Unable to set the max_iter parameter.",
                                    logPrefix);
    return false;
  }

  if (!m_solver->setIntegerParameter("linsys_solver", 0)) {
    BipedalLocomotion::log()->error(
        "{} Unable to set the linsys_solver parameter.", logPrefix);
    return false;
  }

  bool warm_starting{true};
  if (!parametersHandler->getParameter("warm_starting", warm_starting)) {
    BipedalLocomotion::log()->error(
        "{} Unable to find the warm_starting parameter.", logPrefix);
    return false;
  }
  if (!m_solver->setBooleanParameter("warm_starting", warm_starting)) {
    BipedalLocomotion::log()->error(
        "{} Unable to set the warm_starting parameter.", logPrefix);
    return false;
  }

  double absoluteTolerance, relativeTolerance;
  if (!parametersHandler->getParameter("absolute_tolerance",
                                       absoluteTolerance)) {
    BipedalLocomotion::log()->error(
        "{} Unable to find the absolute_tolerance parameter.", logPrefix);
    return false;
  }
  if (!m_solver->setRealNumberParameter("eps_abs", absoluteTolerance)) {
    BipedalLocomotion::log()->error("{} Unable to set the eps_abs parameter.",
                                    logPrefix);
    return false;
  }

  if (!parametersHandler->getParameter("relative_tolerance",
                                       relativeTolerance)) {
    BipedalLocomotion::log()->error(
        "{} Unable to find the relative_tolerance parameter.", logPrefix);
    return false;
  }
  if (!m_solver->setRealNumberParameter("eps_rel", relativeTolerance)) {
    BipedalLocomotion::log()->error("{} Unable to set the eps_rel parameter.",
                                    logPrefix);
    return false;
  }

  return true;
}

bool QPproblem::instantiateProxqpSolver(
    std::shared_ptr<
        const BipedalLocomotion::ParametersHandler::IParametersHandler>
        parametersHandler) {

  constexpr auto logPrefix = "[QPproblem::instantiateOsqpSolver]";

  this->m_solver->instantiateSolver("proxqp");

  bool verbosity{false};
  if (!parametersHandler->getParameter("verbosity", verbosity)) {
    BipedalLocomotion::log()->error(
        "{} Unable to find the verbosity parameter.", logPrefix);
    return false;
  }
  if (!m_solver->setBooleanParameter("verbose", verbosity)) {
    BipedalLocomotion::log()->error("{} Unable to set the verbosity parameter.",
                                    logPrefix);
    return false;
  }

  int maxIterations{1000};
  if (!parametersHandler->getParameter("max_iterations", maxIterations)) {
    BipedalLocomotion::log()->error(
        "{} Unable to find the max_iterations parameter.", logPrefix);
    return false;
  }
  if (!m_solver->setIntegerParameter("max_iter", maxIterations)) {
    BipedalLocomotion::log()->error("{} Unable to set the max_iter parameter.",
                                    logPrefix);
    return false;
  }

  std::string initialGuess;
  if (!parametersHandler->getParameter("initial_guess", initialGuess)) {
    BipedalLocomotion::log()->error(
        "{} Unable to find the initial_guess parameter.", logPrefix);
    return false;
  }
  if (!m_solver->setStringParameter("initial_guess", initialGuess)) {
    BipedalLocomotion::log()->error(
        "{} Unable to set the initial_guess parameter.", logPrefix);
    return false;
  }

  double absoluteTolerance, relativeTolerance;
  if (!parametersHandler->getParameter("absolute_tolerance",
                                       absoluteTolerance)) {
    BipedalLocomotion::log()->error(
        "{} Unable to find the absolute_tolerance parameter.", logPrefix);
    return false;
  }
  if (!m_solver->setRealNumberParameter("eps_abs", absoluteTolerance)) {
    BipedalLocomotion::log()->error("{} Unable to set the eps_abs parameter.",
                                    logPrefix);
    return false;
  }

  if (!parametersHandler->getParameter("relative_tolerance",
                                       relativeTolerance)) {
    BipedalLocomotion::log()->error(
        "{} Unable to find the relative_tolerance parameter.", logPrefix);
    return false;
  }
  if (!m_solver->setRealNumberParameter("eps_rel", relativeTolerance)) {
    BipedalLocomotion::log()->error("{} Unable to set the eps_rel parameter.",
                                    logPrefix);
    return false;
  }

  return true;
}

bool QPproblem::initializeSolver() {

  constexpr auto logPrefix = "[QPproblem::initializeSolver]";

  // Hessian Matrix
  Eigen::SparseMatrix<double> hessianMatrixSparse =
      m_HessianMatrix.sparseView();

  if (!m_solver->data()->setHessianMatrix(hessianMatrixSparse)) {
    BipedalLocomotion::log()->error(
        "{} Unable to set the Hessian matrix for the first time.", logPrefix);
    return false;
  }

  // Gradient
  if (!m_solver->data()->setGradient(m_Gradient)) {
    BipedalLocomotion::log()->error(
        "{} Unable to set the gradient for the first time.", logPrefix);
    return false;
  }

  // Linear Constraints Matrix and Vector
  Eigen::SparseMatrix<double> linearConstraintsMatrixSparse =
      m_linearConstraintsMatrix.sparseView();

  if (!m_solver->data()->setLinearConstraintsMatrix(
          linearConstraintsMatrixSparse)) {
    BipedalLocomotion::log()->error(
        "{} Unable to set the linear constraints matrix for the first time.",
        logPrefix);
    return false;
  }

  if (!m_solver->data()->setUpperBound(m_linearConstraintsVector)) {
    BipedalLocomotion::log()->error(
        "{} Unable to set the upper bound for the first time.", logPrefix);
    return false;
  }

  m_dummyLowerBoundVector =
      Eigen::VectorXd::Constant(m_numberOfConstraints, -QpSolversEigen::INFTY);
  if (!m_solver->data()->setLowerBound(m_dummyLowerBoundVector)) {
    BipedalLocomotion::log()->error(
        "{} Unable to set the lower bound for the first time.", logPrefix);
    return false;
  }

  if (!m_solver->initSolver()) {
    BipedalLocomotion::log()->error("{} Unable to initialize the solver.",
                                    logPrefix);
    return false;
  };

  return m_solver->isInitialized();
};

bool QPproblem::solve() {

  constexpr auto logPrefix = "[QPproblem::solve]";

  if (!m_solver->isInitialized()) {
    if (!this->initializeSolver()) {
      BipedalLocomotion::log()->error("{} Unable to initialize the solver.",
                                      logPrefix);
      return false;
    }
  } else {
    this->updateProblem();
  }

  QpSolversEigen::ErrorExitFlag exitFlag = m_solver->solveProblem();

  if (exitFlag != QpSolversEigen::ErrorExitFlag::NoError) {
    BipedalLocomotion::log()->error("{} Unable to solve the problem.",
                                    logPrefix);
    return false;
  }

  auto solverStatus = m_solver->getStatus();

  if (solverStatus == QpSolversEigen::Status::SolvedInaccurate ||
      solverStatus == QpSolversEigen::Status::SolvedClosestPrimalFeasible) {
    BipedalLocomotion::log()->warn(
        "{} The solver found an inaccurate feasible solution.", logPrefix);
  } else if (solverStatus != QpSolversEigen::Status::Solved) {
    BipedalLocomotion::log()->error(
        "{} The solver was not able to find a feasible solution.", logPrefix);
    m_isSolved = false;
    return false;
  }

  m_isSolved = true;
  return true;
};

void QPproblem::updateProblem() {

  Eigen::SparseMatrix<double> hessianMatrixSparse =
      m_HessianMatrix.sparseView();
  Eigen::SparseMatrix<double> linearConstraintMatrixSparse =
      m_linearConstraintsMatrix.sparseView();

  m_solver->updateHessianMatrix(hessianMatrixSparse);
  m_solver->updateGradient(m_Gradient);
  m_solver->updateLinearConstraintsMatrix(linearConstraintMatrixSparse);
  m_solver->updateUpperBound(m_linearConstraintsVector);
  m_solver->updateLowerBound(m_dummyLowerBoundVector);
}

Eigen::VectorXd QPproblem::getSolution() { return m_solver->getSolution(); };

bool QPproblem::setHessianMatrix(
    const Eigen::Ref<const Eigen::MatrixXd> &Hessian) {
  constexpr auto logPrefix = "[QPproblem::setHessianMatrix]";

  if (!(m_HessianMatrix.size() == Hessian.size())) {
    BipedalLocomotion::log()->error("{} The size of the Hessian matrix must be "
                                    "equal to the number of the variables.",
                                    logPrefix);
    return false;
  }

  m_HessianMatrix = Hessian;
  return true;
}

bool QPproblem::setGradient(const Eigen::Ref<const Eigen::VectorXd> &Gradient) {

  constexpr auto logPrefix = "[QPproblem::setGradient]";

  if (!(m_Gradient.size() == Gradient.size())) {
    BipedalLocomotion::log()->error("{} The size of the gradient must be equal "
                                    "to the number of the variables.",
                                    logPrefix);
    return false;
  }

  m_Gradient = Gradient;
  return true;
}

bool QPproblem::setConstraints(
    const Eigen::Ref<const Eigen::MatrixXd> &linearConstraintsMatrix,
    const Eigen::Ref<const Eigen::VectorXd> &linearConstraintsVector) {

  constexpr auto logPrefix = "[QPproblem::setConstraints]";

  if (!(m_linearConstraintsMatrix.size() == linearConstraintsMatrix.size())) {
    BipedalLocomotion::log()->error(
        "{} The size of the linear constraints "
        "matrix must be equal to the number of the variables.",
        logPrefix);
    return false;
  }

  if (!(m_linearConstraintsVector.size() == linearConstraintsVector.size())) {
    BipedalLocomotion::log()->error(
        "{} The size of the linear constraints "
        "vector must be equal to the number of the variables.",
        logPrefix);
    return false;
  }

  m_linearConstraintsMatrix = linearConstraintsMatrix;
  m_linearConstraintsVector = linearConstraintsVector;

  return true;
}

} // namespace MomentumBasedController