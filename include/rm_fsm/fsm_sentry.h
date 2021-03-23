//
// Created by astro on 2021/1/26.
//

#ifndef SRC_RM_FSM_INCLUDE_RM_FSM_FSM_SENTRY_H_
#define SRC_RM_FSM_INCLUDE_RM_FSM_FSM_SENTRY_H_

#include "rm_fsm/fsm_common.h"
#include "rm_fsm/state_raw.h"
#include "rm_fsm/state_passive.h"
#include "rm_fsm/state_automatic.h"
#include "rm_fsm/state_attack_without_move.h"
template<typename T>
class FsmSentry : public Fsm<T> {
 public:
  explicit FsmSentry(ros::NodeHandle &node_handle);
  std::string getDesiredState();
  StatePassive<T> *state_passive_;
  StateRaw<T> *state_raw_;
  StateAutomatic<T> *state_automatic_;
  StateAttackWithoutMove<T> *state_attack_without_move_;
};

#endif //SRC_RM_FSM_INCLUDE_RM_FSM_FSM_SENTRY_H_
