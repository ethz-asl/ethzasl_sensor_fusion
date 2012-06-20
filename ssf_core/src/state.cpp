/*
 * state.cpp
 *
 *  Created on: Jun 20, 2012
 *      Author: acmarkus
 */

#include <ssf_core/state.h>

namespace ssf_core
{

void State::reset(){
  // states varying during propagation
  p_.setZero();
  v_.setZero();
  q_.setIdentity();
  b_w_.setZero();
  b_a_.setZero();

  L_ = 1.0;
  q_wv_.setIdentity();
  q_ci_.setIdentity();
  p_ic_.setZero();

  w_m_.setZero();
  a_m_.setZero();

  q_int_.setIdentity();

  P_.setZero();
}

}; // end namespace ssf_core
