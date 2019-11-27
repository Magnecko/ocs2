#pragma once

#include <ocs2_core/constraint/ConstraintBase.h>
#include <ocs2_core/cost/CostFunctionBase.h>
#include <ocs2_core/dynamics/ControlledSystemBase.h>
#include <ocs2_core/dynamics/DerivativesBase.h>
#include <ocs2_core/initialization/SystemOperatingPoint.h>
#include <ocs2_core/logic/rules/HybridLogicRules.h>

enum { STATE_DIM = 2, INPUT_DIM = 1 };

namespace ocs2 {

// #######################
// ####LOGIC CLASSES######
// #######################
class system_logic : public HybridLogicRules {
  using logic_rules_t = system_logic;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using BASE = HybridLogicRules;

  system_logic() = default;

  ~system_logic() = default;

  system_logic(scalar_array_t switchingTimes, size_array_t subsystemsSequence)
      : BASE(std::move(switchingTimes), std::move(subsystemsSequence)) {}

  void rewind(const scalar_t& lowerBoundTime, const scalar_t& upperBoundTime) final {}

  void update() final {}

 protected:
  void insertModeSequenceTemplate(const logic_template_type& modeSequenceTemplate, const scalar_t& startTime,
                                  const scalar_t& finalTime) override{};
};

// #######################
// ###DYNAMICS CLASSES####
// #######################
class system_dyn1 : public ControlledSystemBase<STATE_DIM, INPUT_DIM> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  system_dyn1() = default;
  ~system_dyn1() = default;

  void computeFlowMap(const double& t, const Eigen::Vector2d& x, const Eigen::Matrix<double, 1, 1>& u, Eigen::Vector2d& dxdt) {
    Eigen::Matrix<double, STATE_DIM, STATE_DIM> A;
    A << -0.1, 0.9, -1, -0.01;
    Eigen::Matrix<double, STATE_DIM, INPUT_DIM> B;
    B << 0, 1;
    Eigen::Matrix<double, STATE_DIM, 1> F;
    F << 0, 0;

    dxdt = A * x + B * u + F;
  }

  void computeJumpMap(const scalar_t& time, const state_vector_t& state, state_vector_t& mappedState) {
    mappedState[0] = state[0];
    mappedState[1] = state[1];
  }

  void computeGuardSurfaces(const scalar_t& time, const state_vector_t& state, dynamic_vector_t& guardSurfacesValue) {
    guardSurfacesValue = Eigen::Matrix<double, 2, 1>();
    guardSurfacesValue[0] = 1;
    guardSurfacesValue[1] = -state[0] * state[1];
  }

  system_dyn1* clone() const final { return new system_dyn1(*this); }
};

class system_dyn2 : public ControlledSystemBase<STATE_DIM, INPUT_DIM> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  system_dyn2() = default;
  ~system_dyn2() = default;

  void computeFlowMap(const double& t, const Eigen::Vector2d& x, const Eigen::Matrix<double, 1, 1>& u, Eigen::Vector2d& dxdt) {
    Eigen::Matrix<double, STATE_DIM, STATE_DIM> A;
    A << -0, 3, -3, 0;
    Eigen::Matrix<double, STATE_DIM, INPUT_DIM> B;
    B << 0, 1;
    Eigen::Matrix<double, STATE_DIM, 1> F;
    F << 0, 0;

    dxdt = A * x + B * u + F;
  }

  void computeJumpMap(const scalar_t& time, const state_vector_t& state, state_vector_t& mappedState) {
    mappedState[0] = state[0];
    mappedState[1] = state[1];
  }

  void computeGuardSurfaces(const scalar_t& time, const state_vector_t& state, dynamic_vector_t& guardSurfacesValue) {
    guardSurfacesValue = Eigen::Matrix<double, 2, 1>();

    guardSurfacesValue[0] = state[0] * state[1];
    guardSurfacesValue[1] = 1;
  }

  system_dyn2* clone() const final { return new system_dyn2(*this); }
};

class system_dyn : public ControlledSystemBase<STATE_DIM, INPUT_DIM> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Base = ControlledSystemBase<2, 1>;

  system_dyn(std::shared_ptr<system_logic> logicRulesPtr)
      : logicRulesPtr_(std::move(logicRulesPtr)), activeSubsystem_(1), subsystemDynamicsPtr_(2) {
    subsystemDynamicsPtr_[0].reset(new system_dyn1);
    subsystemDynamicsPtr_[1].reset(new system_dyn2);
  }

  ~system_dyn() = default;

  system_dyn* clone() const final { return new system_dyn(*this); }

  void reset() override {
    Base::reset();
    logicRulesPtr_->reset();
  }

  system_dyn(const system_dyn& other) : activeSubsystem_(other.activeSubsystem_), subsystemDynamicsPtr_(2) {
    subsystemDynamicsPtr_[0].reset(other.subsystemDynamicsPtr_[0]->clone());
    subsystemDynamicsPtr_[1].reset(other.subsystemDynamicsPtr_[1]->clone());
    logicRulesPtr_ = other.logicRulesPtr_;
  }

  void computeFlowMap(const scalar_t& t, const state_vector_t& x, const input_vector_t& u, state_vector_t& dxdt) final {
    activeSubsystem_ = logicRulesPtr_->getSubSystemTime(t);
    subsystemDynamicsPtr_[activeSubsystem_]->computeFlowMap(t, x, u, dxdt);
  }

  void computeJumpMap(const scalar_t& time, const state_vector_t& state, state_vector_t& mappedState) override {
    if (activeSubsystem_ == 0) {
      logicRulesPtr_->appendModeSequence(1, time);
    } else {
      logicRulesPtr_->appendModeSequence(0, time);
    }

    activeSubsystem_ = logicRulesPtr_->getSubSystemTime(time);
    subsystemDynamicsPtr_[activeSubsystem_]->computeJumpMap(time, state, mappedState);
  }

  void computeGuardSurfaces(const scalar_t& time, const state_vector_t& state, dynamic_vector_t& guardSurfacesValue) {
    activeSubsystem_ = logicRulesPtr_->getSubSystemTime(time);
    subsystemDynamicsPtr_[activeSubsystem_]->computeGuardSurfaces(time, state, guardSurfacesValue);
  }

 private:
  int activeSubsystem_;
  std::shared_ptr<system_logic> logicRulesPtr_;
  std::vector<Base::Ptr> subsystemDynamicsPtr_;
};

// ############################
// ####DERIVATIVE CLASSES######
// ############################
class system_der_1 : public DerivativesBase<STATE_DIM, INPUT_DIM> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  system_der_1() = default;
  ~system_der_1() = default;

  void getFlowMapDerivativeState(state_matrix_t& A) override { A << -0.1, 0.9, -1, -0.01; }

  void getFlowMapDerivativeInput(state_input_matrix_t& B) override { B << 0, 1; }

  system_der_1* clone() const override { return new system_der_1(*this); }
};

class system_der_2 : public DerivativesBase<STATE_DIM, INPUT_DIM> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  system_der_2() = default;
  ~system_der_2() = default;

  void getFlowMapDerivativeState(state_matrix_t& A) override { A << -0, 3, -3, 0; }

  void getFlowMapDerivativeInput(state_input_matrix_t& B) override { B << 0, 1; }

  system_der_2* clone() const override { return new system_der_2(*this); }
};

class system_der : public DerivativesBase<STATE_DIM, INPUT_DIM> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Base = DerivativesBase<STATE_DIM, INPUT_DIM>;

  system_der(std::shared_ptr<system_logic> logicRulesPtr)
      : logicRulesPtr_(std::move(logicRulesPtr)), activeSubsystem_(1), subsystemDerPtr_(2) {
    subsystemDerPtr_[0].reset(new system_der_1);
    subsystemDerPtr_[1].reset(new system_der_2);
  }

  ~system_der() = default;

  system_der* clone() const final { return new system_der(*this); }

  system_der(const system_der& other) : activeSubsystem_(other.activeSubsystem_), subsystemDerPtr_(2) {
    subsystemDerPtr_[0].reset(other.subsystemDerPtr_[0]->clone());
    subsystemDerPtr_[1].reset(other.subsystemDerPtr_[1]->clone());
    logicRulesPtr_ = other.logicRulesPtr_;
  }

  void setCurrentStateAndControl(const scalar_t& t, const state_vector_t& x, const input_vector_t& u) final {
    Base::setCurrentStateAndControl(t, x, u);
    activeSubsystem_ = logicRulesPtr_->getSubSystemTime(t);
    subsystemDerPtr_[activeSubsystem_]->setCurrentStateAndControl(t, x, u);
  }

  void getFlowMapDerivativeState(state_matrix_t& A) final { subsystemDerPtr_[activeSubsystem_]->getFlowMapDerivativeState(A); }

  void getFlowMapDerivativeInput(state_input_matrix_t& B) final { subsystemDerPtr_[activeSubsystem_]->getFlowMapDerivativeInput(B); }

 private:
  int activeSubsystem_;
  std::shared_ptr<system_logic> logicRulesPtr_;
  std::vector<Base::Ptr> subsystemDerPtr_;
};

// #######################
// #### COST CLASSES######
// #######################
class system_cost_1 : public CostFunctionBase<STATE_DIM, INPUT_DIM> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  system_cost_1() = default;
  ~system_cost_1() = default;

  system_cost_1* clone() const final { return new system_cost_1(*this); }

  void getIntermediateCost(scalar_t& L) final { L = 0.5 * pow(x_[0], 2) + 0.5 * pow(x_[1], 2) + 0.005 * pow(u_[0], 2); }

  void getIntermediateCostDerivativeState(state_vector_t& dLdx) final { dLdx << x_[0], x_[1]; }

  void getIntermediateCostSecondDerivativeState(state_matrix_t& dLdxx) final { dLdxx << 1.0, 0.0, 1.0, 0.0; }

  void getIntermediateCostDerivativeInput(input_vector_t& dLdu) final { dLdu << 0.01 * u_[0]; }

  void getIntermediateCostSecondDerivativeInput(input_matrix_t& dLduu) final { dLduu << 0.01; }

  void getIntermediateCostDerivativeInputState(input_state_matrix_t& dLdxu) final { dLdxu.setZero(); }

  /*
  Terminal Cost Functions
   */
  void getTerminalCost(scalar_t& Phi) final { Phi = 0.5 * pow(x_[0], 2) + 0.5 * pow(x_[1], 2); }
  void getTerminalCostDerivativeState(state_vector_t& dPhidx) final { dPhidx << x_[0], x_[1]; }
  void getTerminalCostSecondDerivativeState(state_matrix_t& dPhidxx) final { dPhidxx << 1.0, 0.0, 1.0, 0.0; }
};

class system_cost_2 : public CostFunctionBase<STATE_DIM, INPUT_DIM> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  system_cost_2() = default;
  ~system_cost_2() = default;

  system_cost_2* clone() const final { return new system_cost_2(*this); }

  void getIntermediateCost(scalar_t& L) final { L = 0.5 * pow(x_[0], 2) + 0.5 * pow(x_[1], 2) + 0.005 * pow(u_[0], 2); }

  void getIntermediateCostDerivativeState(state_vector_t& dLdx) final { dLdx << x_[0], x_[1]; }

  void getIntermediateCostSecondDerivativeState(state_matrix_t& dLdxx) final { dLdxx << 1.0, 0.0, 1.0, 0.0; }

  void getIntermediateCostDerivativeInput(input_vector_t& dLdu) final { dLdu << 0.01 * u_[0]; }

  void getIntermediateCostSecondDerivativeInput(input_matrix_t& dLduu) final { dLduu << 0.01; }

  void getIntermediateCostDerivativeInputState(input_state_matrix_t& dLdxu) final { dLdxu.setZero(); }

  /*
  Terminal Cost Functions
   */
  void getTerminalCost(scalar_t& Phi) final { Phi = 0.5 * pow(x_[0], 2) * pow(x_[1], 2); }
  void getTerminalCostDerivativeState(state_vector_t& dPhidx) final { dPhidx << x_[0], x_[1]; }
  void getTerminalCostSecondDerivativeState(state_matrix_t& dPhidxx) final { dPhidxx << 1.0, 0.0, 1.0, 0.0; }
};

class system_cost : public CostFunctionBase<STATE_DIM, INPUT_DIM> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Base = CostFunctionBase<STATE_DIM, INPUT_DIM>;

  system_cost(std::shared_ptr<system_logic> logicRulesPtr)
      : logicRulesPtr_(std::move(logicRulesPtr)), activeSubsystem_(1), subsystemCostPtr_(2) {
    subsystemCostPtr_[0].reset(new system_cost_1);
    subsystemCostPtr_[1].reset(new system_cost_2);
  }

  system_cost* clone() const final { return new system_cost(*this); }

  ~system_cost() = default;

  system_cost(const system_cost& other) : activeSubsystem_(other.activeSubsystem_), subsystemCostPtr_(2) {
    subsystemCostPtr_[0].reset(other.subsystemCostPtr_[0]->clone());
    subsystemCostPtr_[1].reset(other.subsystemCostPtr_[1]->clone());
    logicRulesPtr_ = other.logicRulesPtr_;
  }

  void setCurrentStateAndControl(const scalar_t& t, const state_vector_t& x, const input_vector_t& u) {
    t_ = t;
    x_ = x;
    u_ = u;

    activeSubsystem_ = logicRulesPtr_->getSubSystemTime(t_);
    subsystemCostPtr_[activeSubsystem_]->setCurrentStateAndControl(t, x, u);
  }
  /*
   * Intermediate cost function
   */

  void getIntermediateCost(scalar_t& L) final {
    activeSubsystem_ = logicRulesPtr_->getSubSystemTime(t_);
    subsystemCostPtr_[activeSubsystem_]->getIntermediateCost(L);
  }

  void getIntermediateCostDerivativeState(state_vector_t& dLdx) final {
    activeSubsystem_ = logicRulesPtr_->getSubSystemTime(t_);
    subsystemCostPtr_[activeSubsystem_]->getIntermediateCostDerivativeState(dLdx);
  }

  void getIntermediateCostSecondDerivativeState(state_matrix_t& dLdxx) final {
    activeSubsystem_ = logicRulesPtr_->getSubSystemTime(t_);
    subsystemCostPtr_[activeSubsystem_]->getIntermediateCostSecondDerivativeState(dLdxx);
  }

  void getIntermediateCostDerivativeInput(input_vector_t& dLdu) final {
    activeSubsystem_ = logicRulesPtr_->getSubSystemTime(t_);
    subsystemCostPtr_[activeSubsystem_]->getIntermediateCostDerivativeInput(dLdu);
  }

  void getIntermediateCostSecondDerivativeInput(input_matrix_t& dLduu) final {
    activeSubsystem_ = logicRulesPtr_->getSubSystemTime(t_);
    subsystemCostPtr_[activeSubsystem_]->getIntermediateCostSecondDerivativeInput(dLduu);
  }

  void getIntermediateCostDerivativeInputState(input_state_matrix_t& dLdxu) final {
    activeSubsystem_ = logicRulesPtr_->getSubSystemTime(t_);
    subsystemCostPtr_[activeSubsystem_]->getIntermediateCostDerivativeInputState(dLdxu);
  }

  /*
          Terminal Cost Functions
   */
  void getTerminalCost(scalar_t& Phi) final {
    activeSubsystem_ = logicRulesPtr_->getSubSystemTime(t_);
    subsystemCostPtr_[activeSubsystem_]->getTerminalCost(Phi);
  }

  void getTerminalCostDerivativeState(state_vector_t& dPhidx) final {
    activeSubsystem_ = logicRulesPtr_->getSubSystemTime(t_);
    subsystemCostPtr_[activeSubsystem_]->getTerminalCostDerivativeState(dPhidx);
  }

  void getTerminalCostSecondDerivativeState(state_matrix_t& dPhidxx) final {
    activeSubsystem_ = logicRulesPtr_->getSubSystemTime(t_);
    subsystemCostPtr_[activeSubsystem_]->getTerminalCostSecondDerivativeState(dPhidxx);
  }

 private:
  int activeSubsystem_;
  std::shared_ptr<system_logic> logicRulesPtr_;
  std::vector<Base::Ptr> subsystemCostPtr_;
};
// #############################
// #### CONSTRAINT CLASSES######
// #############################
class system_const_1 : public ConstraintBase<STATE_DIM, INPUT_DIM> {
 public:
  system_const_1() = default;
  ~system_const_1() = default;

  system_const_1* clone() const final { return new system_const_1(*this); }

  void getInequalityConstraint(scalar_array_t& h) override {
    h.resize(4);
    h[0] = -u_[0] + 10;
    h[1] = u_[0] + 10;
    h[2] = x_[0] + 50;
    h[3] = -x_[0] + 50;
  }

  size_t numInequalityConstraint(const scalar_t& time) override { return 4; }

  void getInequalityConstraintDerivativesState(state_vector_array_t& dhdx) override {
    dhdx.resize(4);
    dhdx[0].setZero();
    dhdx[1].setZero();
    dhdx[2] << 1, 0;
    dhdx[3] << -1, 0;
  }

  void getInequalityConstraintDerivativesInput(input_vector_array_t& dhdu) override {
    dhdu.resize(4);
    dhdu[0] << -1;
    dhdu[1] << 1;
    dhdu[2] << 0;
    dhdu[3] << 0;
  }

  void getInequalityConstraintSecondDerivativesState(state_matrix_array_t& ddhdxdx) override {
    ddhdxdx.resize(4);
    ddhdxdx[0].setZero();
    ddhdxdx[1].setZero();
    ddhdxdx[2].setZero();
    ddhdxdx[3].setZero();
  }

  void getInequalityConstraintSecondDerivativesInput(input_matrix_array_t& ddhdudu) override {
    ddhdudu.resize(4);
    ddhdudu[0].setZero();
    ddhdudu[1].setZero();
    ddhdudu[2].setZero();
    ddhdudu[3].setZero();
  }
  void getInequalityConstraintDerivativesInputState(input_state_matrix_array_t& ddhdudx) override {
    ddhdudx.resize(4);
    ddhdudx[0].setZero();
    ddhdudx[1].setZero();
    ddhdudx[2].setZero();
    ddhdudx[3].setZero();
  }
};

class system_const_2 : public ConstraintBase<STATE_DIM, INPUT_DIM> {
 public:
  system_const_2() = default;
  ~system_const_2() = default;

  system_const_2* clone() const final { return new system_const_2(*this); }

  void getInequalityConstraint(scalar_array_t& h) override {
    h.resize(4);
    h[0] = -u_[0] + 10;
    h[1] = u_[0] + 10;
    h[2] = x_[0] + 50;
    h[3] = -x_[0] + 50;
  }

  size_t numInequalityConstraint(const scalar_t& time) override { return 4; }

  void getInequalityConstraintDerivativesState(state_vector_array_t& dhdx) override {
    dhdx.resize(4);
    dhdx[0].setZero();
    dhdx[1].setZero();
    dhdx[2] << 1, 0;
    dhdx[3] << -1, 0;
  }

  void getInequalityConstraintDerivativesInput(input_vector_array_t& dhdu) override {
    dhdu.resize(4);
    dhdu[0] << -1;
    dhdu[1] << 1;
    dhdu[2] << 0;
    dhdu[3] << 0;
  }

  void getInequalityConstraintSecondDerivativesState(state_matrix_array_t& ddhdxdx) override {
    ddhdxdx.resize(4);
    ddhdxdx[0].setZero();
    ddhdxdx[1].setZero();
    ddhdxdx[2].setZero();
    ddhdxdx[3].setZero();
  }

  void getInequalityConstraintSecondDerivativesInput(input_matrix_array_t& ddhdudu) override {
    ddhdudu.resize(4);
    ddhdudu[0].setZero();
    ddhdudu[1].setZero();
    ddhdudu[2].setZero();
    ddhdudu[3].setZero();
  }
  void getInequalityConstraintDerivativesInputState(input_state_matrix_array_t& ddhdudx) override {
    ddhdudx.resize(4);
    ddhdudx[0].setZero();
    ddhdudx[1].setZero();
    ddhdudx[2].setZero();
    ddhdudx[3].setZero();
  }
};

class system_const : public ConstraintBase<STATE_DIM, INPUT_DIM> {
 public:
  using Base = ConstraintBase<STATE_DIM, INPUT_DIM>;

  system_const(std::shared_ptr<system_logic> logicRulesPtr)
      : logicRulesPtr_(std::move(logicRulesPtr)), activeSubsystem_(1), subsystemConstPtr_(2) {
    subsystemConstPtr_[0].reset(new system_const_1);
    subsystemConstPtr_[1].reset(new system_const_2);
  }

  ~system_const() = default;

  virtual void setCurrentStateAndControl(const scalar_t& t, const state_vector_t& x, const input_vector_t& u) {
    t_ = t;
    x_ = x;
    u_ = u;

    activeSubsystem_ = logicRulesPtr_->getSubSystemTime(t_);
    subsystemConstPtr_[activeSubsystem_]->setCurrentStateAndControl(t_, x_, u_);
  }

  void getInequalityConstraint(scalar_array_t& h) override { subsystemConstPtr_[activeSubsystem_]->getInequalityConstraint(h); }

  size_t numInequalityConstraint(const scalar_t& time) override {
    return subsystemConstPtr_[activeSubsystem_]->numInequalityConstraint(time);
  }

  void getInequalityConstraintDerivativesState(state_vector_array_t& dhdx) override {
    subsystemConstPtr_[activeSubsystem_]->getInequalityConstraintDerivativesState(dhdx);
  }

  void getInequalityConstraintDerivativesInput(input_vector_array_t& dhdu) override {
    subsystemConstPtr_[activeSubsystem_]->getInequalityConstraintDerivativesInput(dhdu);
  }

  void getInequalityConstraintSecondDerivativesInput(input_matrix_array_t& ddhdudu) override {
    subsystemConstPtr_[activeSubsystem_]->getInequalityConstraintSecondDerivativesInput(ddhdudu);
  }

  void getInequalityConstraintSecondDerivativesState(state_matrix_array_t& ddhdxdx) override {
    subsystemConstPtr_[activeSubsystem_]->getInequalityConstraintSecondDerivativesState(ddhdxdx);
  }

  void getInequalityConstraintDerivativesInputState(input_state_matrix_array_t& ddhdudx) override {
    subsystemConstPtr_[activeSubsystem_]->getInequalityConstraintDerivativesInputState(ddhdudx);
  }

  system_const* clone() const final { return new system_const(*this); }

 private:
  int activeSubsystem_;
  std::shared_ptr<system_logic> logicRulesPtr_;
  std::vector<Base::Ptr> subsystemConstPtr_;
};

using system_op = SystemOperatingPoint<STATE_DIM, INPUT_DIM>;
}  // namespace ocs2
