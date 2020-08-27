/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#pragma once

#include <iostream>
#include <string>

#include <ocs2_core/Types.h>

namespace ocs2 {
namespace mpc {

/**
 * This structure holds all setting parameters for the MPC class.
 */
struct Settings {
  /** MPC time horizon lenght. */
  scalar_t timeHorizon_ = 1.0;
  /** Number of data partitions over the time horizon. */
  size_t numPartitions_ = 2;

  /** Number of iterations which will be used during MPC regular loop. */
  size_t runtimeMaxNumIterations_ = 15;
  /** Number of iterations which will be used during MPC initial run. */
  size_t initMaxNumIterations_ = 15;

  /** Maximum learning rate which will be used during MPC regular loop. */
  scalar_t runtimeMaxStepLength_ = 1.0;
  /** Maximum learning rate which will be used during MPC regular loop. */
  scalar_t runtimeMinStepLength_ = 1.0;
  /** Maximum learning rate which will be used during MPC initial run. */
  scalar_t initMaxStepLength_ = 1.0;
  /** Minimum learning rate which will be used during MPC initial run. */
  scalar_t initMinStepLength_ = 1.0;

  /** This value determines to initialize the SLQ with the controller from previous call
   * (warm start) or the given operating trajectories (cold start). */
  bool coldStart_ = false;

  /** The time window (in seconds) for retrieving the optimized output (controller and trajectory). */
  scalar_t solutionTimeWindow_ = -1;

  /** This value determines to display the log output of MPC. */
  bool debugPrint_ = false;

  /**
   * MPC loop frequency in Hz. This setting is only used in Dummy_Loop for testing.
   * If set to a positive number, MPC loop of test will be simulated to run by this
   * frequency. Note that this might not be the MPC's realtime frequency.
   */
  scalar_t mpcDesiredFrequency_ = -1;
  /**
   * MRT loop frequency in Hz. This setting is only used in Dummy_Loop for testing.
   * This should always set to a positive number which can be interpreted as the
   * tracking controller's frequency.
   */
  scalar_t mrtDesiredFrequency_ = 100.0;
};

/**
 * Loads the MPC settings from a given file.
 *
 * @param [in] filename: File name which contains the configuration data.
 * @param [in] fieldName: Field name which contains the configuration data.
 * @param [in] verbose: Flag to determine whether to print out the loaded settings or not.
 * @return The MPC settings
 */
Settings loadSettings(const std::string& filename, const std::string& fieldName = "mpc", bool verbose = true);

}  // namespace mpc
}  // namespace ocs2
