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

#include <ocs2_core/Types.h>
#include <ocs2_oc/oc_data/PerformanceIndex.h>
#include <ocs2_oc/oc_data/PrimalSolution.h>

#include "ocs2_sqp/MultipleShootingSolverStatus.h"
#include "ocs2_sqp/TimeDiscretization.h"

namespace ocs2 {
namespace multiple_shooting {

/** Compute 2-norm of the trajectory: sqrt(sum_i v[i]^2) */
inline scalar_t trajectoryNorm(const vector_array_t& v) {
  scalar_t norm = 0.0;
  for (const auto& vi : v) {
    norm += vi.squaredNorm();
  }
  return std::sqrt(norm);
}

/** Increment the given trajectory as: vNew[i] = v[i] + alpha * dv[i]. It assumes that vNew is already resized to size of v. */
template <typename Type>
void incrementTrajectory(const std::vector<Type>& v, const std::vector<Type>& dv, const scalar_t alpha, std::vector<Type>& vNew) {
  assert(v.size() == dv.size());
  if (v.size() != vNew.size()) {
    throw std::runtime_error("[incrementTrajectory] Resize vNew to the size of v!");
  }

  for (int i = 0; i < v.size(); i++) {
    if (dv[i].size() > 0) {  // account for absence of inputs at events.
      vNew[i] = v[i] + alpha * dv[i];
    } else {
      vNew[i] = Type();
    }
  }
}

/**
 * Computes the Armijo descent metric that determines if the solution is a descent direction for the cost. It calculates sum of
 * gradient(cost).dot([dx; du]) over the trajectory.
 *
 * @param [in] cost: The quadratic approximation of the cost.
 * @param [in] deltaXSol: The state trajectory of the QP subproblem solution.
 * @param [in] deltaUSol: The input trajectory of the QP subproblem solution.
 * @return The Armijo descent metric.
 */
scalar_t armijoDescentMetric(const std::vector<ScalarFunctionQuadraticApproximation>& cost, const vector_array_t& deltaXSol,
                             const vector_array_t& deltaUSol);

/**
 * Re-map the projected input back to the original space.
 *
 * @param [in] constraintsProjection: The constraints projection.
 * @param [in] deltaXSol: The state trajectory of the QP subproblem solution.
 * @param [in, out] deltaUSol: The input trajectory of the QP subproblem solution.
 */
void remapProjectedInput(const std::vector<VectorFunctionLinearApproximation>& constraintsProjection, const vector_array_t& deltaXSol,
                         vector_array_t& deltaUSol);

void remapProjectedGain(const std::vector<VectorFunctionLinearApproximation>& constraintsProjection, matrix_array_t& KMatrices);

/**
 * Constructs a primal solution (with a feedforward controller) based the LQ subproblem solution.
 *
 * @param [in] time : The annotated time trajectory
 * @param [in] modeSchedule: The mode schedule.
 * @param [in] x: The state trajectory of the QP subproblem solution.
 * @param [in] u: The input trajectory of the QP subproblem solution.
 * @return The primal solution.
 */
PrimalSolution toPrimalSolution(const std::vector<AnnotatedTime>& time, ModeSchedule&& modeSchedule, vector_array_t&& x,
                                vector_array_t&& u);

/**
 * Constructs a primal solution (with a linear controller) based the LQ subproblem solution.
 *
 * @param [in] time : The annotated time trajectory
 * @param [in] modeSchedule: The mode schedule.
 * @param [in] x: The state trajectory of the QP subproblem solution.
 * @param [in] u: The input trajectory of the QP subproblem solution.
 * @param [in] KMatrices: The LQR gain trajectory of the QP subproblem solution.
 * @return The primal solution.
 */
PrimalSolution toPrimalSolution(const std::vector<AnnotatedTime>& time, ModeSchedule&& modeSchedule, vector_array_t&& x, vector_array_t&& u,
                                matrix_array_t&& KMatrices);

/**
 * Coefficients to compute the Newton step of the Lagrange multiplier associated with the state-input equality constraint such that
 * dfdx*dx + dfdu*du + dfdcostate*dcostate + f
 */
struct ProjectionMultiplierCoefficients {
  matrix_t dfdx;
  matrix_t dfdu;
  matrix_t dfdcostate;
  vector_t f;

  /**
   * Extracts the coefficients to compute the Newton step of the Lagrange multiplier associated with the state-input equality constraint.
   *
   * @param dynamics : Dynamics
   * @param cost : Cost
   * @param constraintProjection : Constraint projection.
   * @param pseudoInverse : Left pseudo-inverse of D^T of the state-input equality constraint.
   */
  void extractProjectionMultiplierCoefficients(const VectorFunctionLinearApproximation& dynamics,
                                               const ScalarFunctionQuadraticApproximation& cost,
                                               const VectorFunctionLinearApproximation& constraintProjection,
                                               const matrix_t& pseudoInverse);
};

}  // namespace multiple_shooting
}  // namespace ocs2
