import math
import numpy as np

from common.numpy_fast import clip
from common.op_params import opParams
from common.realtime import DT_CTRL
from cereal import log
from selfdrive.controls.lib.drive_helpers import get_steer_max


class LatControlLQR():
  def __init__(self, CP):
    self.scale = CP.lateralTuning.lqr.scale
    self.ki = CP.lateralTuning.lqr.ki

    self.A = np.array(CP.lateralTuning.lqr.a).reshape((2, 2))
    self.B = np.array(CP.lateralTuning.lqr.b).reshape((2, 1))
    self.C = np.array(CP.lateralTuning.lqr.c).reshape((1, 2))
    self.K = np.array(CP.lateralTuning.lqr.k).reshape((1, 2))
    self.L = np.array(CP.lateralTuning.lqr.l).reshape((2, 1))
    self.dc_gain = CP.lateralTuning.lqr.dcGain

    self.x_hat = np.array([[0], [0]])
    self.i_unwind_rate = 0.3 * DT_CTRL
    self.i_rate = 1.0 * DT_CTRL

    self.sat_count_rate = 1.0 * DT_CTRL
    self.sat_limit = CP.steerLimitTimer
    
    self._op_params = opParams(calling_function="latcontrol_lqr.py")
    self.roll_k = 1.0

    self.reset()
  
  def update_op_params(self):
    self.scale = self._op_params.get('TUNE_LAT_LQR_scale')
    self.ki = self._op_params.get('TUNE_LAT_LQR_ki')
    self.dc_gain = self._op_params.get('TUNE_LAT_LQR_dc_gain')
    a = self._op_params.get('TUNE_LAT_LQR_a')
    b = self._op_params.get('TUNE_LAT_LQR_b')
    c = self._op_params.get('TUNE_LAT_LQR_c')
    k = self._op_params.get('TUNE_LAT_LQR_k')
    l = self._op_params.get('TUNE_LAT_LQR_l')
    self.A = np.array(a).reshape((2, 2))
    self.B = np.array(b).reshape((2, 1))
    self.C = np.array(c).reshape((1, 2))
    self.K = np.array(k).reshape((1, 2))
    self.L = np.array(l).reshape((2, 1))
    self.roll_k = self._op_params.get('TUNE_LAT_LQR_roll_compensation')

  def reset(self):
    self.i_lqr = 0.0
    self.sat_count = 0.0

  def _check_saturation(self, control, check_saturation, limit):
    saturated = abs(control) == limit

    if saturated and check_saturation:
      self.sat_count += self.sat_count_rate
    else:
      self.sat_count -= self.sat_count_rate

    self.sat_count = clip(self.sat_count, 0.0, 1.0)

    return self.sat_count > self.sat_limit

  def update(self, active, CS, CP, VM, params, desired_curvature, desired_curvature_rate, llk = None, mean_curvature=0.0):
    lqr_log = log.ControlsState.LateralLQRState.new_message()

    steers_max = get_steer_max(CP, CS.vEgo)
    torque_scale = (0.45 + CS.vEgo / 60.0)**2  # Scale actuator model with speed

    # Subtract offset. Zero angle should correspond to zero torque
    steering_angle_no_offset = CS.steeringAngleDeg - params.angleOffsetAverageDeg

    desired_angle = math.degrees(VM.get_steer_from_curvature(-desired_curvature, CS.vEgo, params.roll * self.roll_k))

    instant_offset = params.angleOffsetDeg - params.angleOffsetAverageDeg
    desired_angle += instant_offset  # Only add offset that originates from vehicle model errors

    # Update Kalman filter
    angle_steers_k = float(self.C.dot(self.x_hat))
    e = steering_angle_no_offset - angle_steers_k
    self.x_hat = self.A.dot(self.x_hat) + self.B.dot(CS.steeringTorqueEps / torque_scale) + self.L.dot(e)

    if CS.vEgo < 0.3 or not active:
      lqr_log.active = False
      lqr_output = 0.
      output_steer = 0.
      self.reset()
    else:
      lqr_log.active = True

      # LQR
      u_lqr = float(desired_angle / self.dc_gain - self.K.dot(self.x_hat))
      lqr_output = torque_scale * u_lqr / self.scale

      # Integrator
      if CS.steeringPressed:
        self.i_lqr -= self.i_unwind_rate * float(np.sign(self.i_lqr))
      else:
        error = desired_angle - angle_steers_k
        i = self.i_lqr + self.ki * self.i_rate * error
        control = lqr_output + i

        if (error >= 0 and (control <= steers_max or i < 0.0)) or \
           (error <= 0 and (control >= -steers_max or i > 0.0)):
          self.i_lqr = i

      output_steer = lqr_output + self.i_lqr
      output_steer = clip(output_steer, -steers_max, steers_max)

    check_saturation = (CS.vEgo > 10) and not CS.steeringRateLimited and not CS.steeringPressed
    saturated = self._check_saturation(output_steer, check_saturation, steers_max)

    lqr_log.steeringAngleDeg = angle_steers_k + params.angleOffsetAverageDeg
    lqr_log.i = self.i_lqr
    lqr_log.output = output_steer
    lqr_log.lqrOutput = lqr_output
    lqr_log.saturated = saturated
    return output_steer, desired_angle, lqr_log
