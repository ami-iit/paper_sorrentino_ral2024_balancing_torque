/**
 * @file main.cpp
 * @authors Lorenzo Moretti, Ines Sorrentino
 * @copyright 2024 Istituto Italiano di Tecnologia (IIT). This software may be
 * modified and distributed under the terms of the BSD-3 clause license.
 */

#include <cstdlib>

#include <memory>
#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>

#include <BipedalLocomotion/ParametersHandler/YarpImplementation.h>
#include <BipedalLocomotion/System/AdvanceableRunner.h>
#include <BipedalLocomotion/System/Barrier.h>
#include <BipedalLocomotion/System/Clock.h>
#include <BipedalLocomotion/System/QuitHandler.h>
#include <BipedalLocomotion/System/YarpClock.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

#include <MomentumBasedTorqueControl/Controller.h>

int main(int argc, char *argv[]) {
  constexpr auto logPrefix = "[main]";

  BipedalLocomotion::System::ClockBuilder::setFactory(
      std::make_shared<BipedalLocomotion::System::YarpClockFactory>());

  // initialize yarp network
  yarp::os::Network yarp;
  if (!yarp.checkNetwork()) {
    BipedalLocomotion::log()->error("[main] Unable to find YARP network.");
    return EXIT_FAILURE;
  }

  // prepare and configure the resource finder
  yarp::os::ResourceFinder &rf =
      yarp::os::ResourceFinder::getResourceFinderSingleton();

  BipedalLocomotion::log()->info(
      "file found: {}",
      rf.findFileByName("blf-momentum-based-torque-control-options."
                        "ini"));
  if (rf.findFileByName("blf-momentum-based-torque-control-options.ini")
          .empty()) {
    BipedalLocomotion::log()->warn(
        "{} Unable to find the default configuration file.", logPrefix);
  }
  rf.setDefaultConfigFile("blf-momentum-based-torque-control-options.ini");
  rf.configure(argc, argv);
  auto handler = std::make_shared<
      BipedalLocomotion::ParametersHandler::YarpImplementation>();
  handler->set(rf);

  // create shared resources
  auto controllerEmptyInput = BipedalLocomotion::System::SharedResource<
      BipedalLocomotion::System::EmptySignal>::create();
  auto controllerEmptyOutput = BipedalLocomotion::System::SharedResource<
      BipedalLocomotion::System::EmptySignal>::create();

  // Instantiate the controller
  auto controller =
      std::make_unique<MomentumBasedController::WholeBodyController>();

  // Initialize the controller
  if (!controller->initialize(handler)) {
    BipedalLocomotion::log()->error("{} Unable to initialize the controller.",
                                    logPrefix);
    return EXIT_FAILURE;
  }

  // Instantiate and initialize the controller runner
  BipedalLocomotion::System::AdvanceableRunner<
      MomentumBasedController::WholeBodyController>
      controllerRunner;

  if (!controllerRunner.initialize(handler->getGroup("WHOLE_BODY_RUNNER"))) {
    BipedalLocomotion::log()->error(
        "{} Unable to initialize the whole body runner.", logPrefix);
    return EXIT_FAILURE;
  }

  controllerRunner.setInputResource(controllerEmptyInput);
  controllerRunner.setOutputResource(controllerEmptyOutput);
  controllerRunner.setAdvanceable(std::move(controller));

  // handle the ctrl+c signal
  BipedalLocomotion::System::handleQuitSignals(
      [&]() { controllerRunner.stop(); });

  // run the controller
  auto controllerThread = controllerRunner.run();

  // wait for the controller thread to finish
  while (controllerRunner.isRunning()) {
    using namespace std::chrono_literals;

    BipedalLocomotion::clock().yield();
    BipedalLocomotion::clock().sleepFor(200ms);
  }

  // stop the controller thread
  controllerRunner.stop();

  // join the controller thread
  if (controllerThread.joinable()) {
    controllerThread.join();
    controllerThread = std::thread();
  }

  return EXIT_SUCCESS;
}