
/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#include <cola2_control/low_level_controllers/pid.h>

PID::PID(std::string name)
  : IController(name)
  , kp_(0)
  , ti_(0)
  , td_(0)
  , i_limit_(0)
  , fff_(0)
  , time_old_(0)
  , feedback_old_(0)
  , error_old_(0)
  , eik_old_(0)
  , derivative_term_from_feedback_(true)
  , edotk_old_(0)
{
}

void PID::reset()
{
  eik_old_ = 0.0;
  time_old_ = 0;
}

double PID::compute(double time_in_sec, double setpoint, double feedback)
{
  // std::cout << "Compute " << _name << ", time: " << time_in_sec << ", setpoint: " << setpoint << ", feedback: "
  // << feedback << std::endl;

  // Check time
  if (time_old_ == 0)
  {
    // Fist time or just reset
    time_old_ = time_in_sec;
    feedback_old_ = feedback;
    error_old_ = setpoint - feedback;
    edotk_old_ = 0.0;
    return 0.0;
  }

  double dt = time_in_sec - time_old_;
  if (dt > 0.2)
  {
    // To much time without controlling this DoF, reset it!
    std::cout << "ERROR wit PID " << name_ << " dt = " << dt << "\n";
    reset();
    return 0.0;
  }

  if (dt < 0.01)
    dt = 0.01;
  time_old_ = time_in_sec;
  // std::cout << "PID " << name_ << " dt: " << dt << std::endl;

  // Compute error, derivative of feedback and integral part
  double ek = setpoint - feedback;
  // std::cout << "PID "  << name_ << " ek: " << ek << std::endl;

  double edotk = 0.0;
  if (derivative_term_from_feedback_)
  {
    edotk = -(feedback - feedback_old_) / dt;
  }
  else
  {
    edotk = (ek - error_old_) / dt;
  }

  // std::cout << "edotk: " << edotk << "\n";
  // std::cout << "tD: " << td_ << "\n";
  // std::cout << "part derivativa: " << kp_ * td_ * edotk << "\n";

  // edotk = (edotk + edotk_old_) / 2.0;
  // std::cout << "edotk filtered: " << edotk << "\n";

  double eik = eik_old_ + (ek * dt);

  // Compute the integral part if ti > 0
  double integral_part = 0.0;
  double tau = 0;
  if (ti_ > 0.0)
  {
    // Integral part
    integral_part = (kp_ / ti_) * eik;

    // Saturate integral part
    // (anti-windup condition, integral part not higher than a value)
    integral_part = cola2::utils::saturate(integral_part, i_limit_);

    // Restore eik
    if (kp_ > 0.0)
    {
      // Avoid division by zero
      eik = integral_part * ti_ / kp_;
    }
    else
    {
      eik = 0.0;
    }
    // Compute tau
    tau = kp_ * (ek + td_ * edotk) + integral_part + fff_;
  }
  else
  {
    // Compute tau without integral part
    tau = kp_ * (ek + td_ * edotk) + fff_;
  }
  // std::cout << "PID " << name_ << " tau: " << tau << std::endl;

  // Store for the next time
  feedback_old_ = feedback;
  error_old_ = ek;
  eik_old_ = eik;
  edotk_old_ = edotk;

  // Return saturate tau
  return cola2::utils::saturate(tau, 1.0);
}

bool PID::setParameters(std::map<std::string, double> params)
{
  std::cout << "Set params for " << name_ << " as: " << params["kp"] << ", " << params["ti"] << ", " << params["td"]
            << ", " << params["derivative_term_from_feedback"] << "\n";

  try
  {
    kp_ = params["kp"];
    ti_ = params["ti"];
    td_ = params["td"];
    i_limit_ = params["i_limit"];
    fff_ = params["fff"];
    derivative_term_from_feedback_ = static_cast<bool>(params["derivative_term_from_feedback"]);
  }
  catch (...)
  {
    std::cout << "PID " << name_ << " setting parameters ERROR! \n";
    return false;
  }
  return true;
}
