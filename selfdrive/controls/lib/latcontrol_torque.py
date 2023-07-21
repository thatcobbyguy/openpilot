import copy
from collections import deque
import math

from cereal import log
from common.filter_simple import FirstOrderFilter
from common.numpy_fast import interp, clip
from selfdrive.controls.lib.drive_helpers import CONTROL_N, apply_deadzone
from selfdrive.controls.lib.latcontrol import LatControl
from selfdrive.controls.lib.pid import PIDController
from selfdrive.controls.lib.vehicle_model import ACCELERATION_DUE_TO_GRAVITY
from selfdrive.modeld.constants import T_IDXS
from system.swaglog import cloudlog

# At higher speeds (25+mph) we can assume:
# Lateral acceleration achieved by a specific car correlates to
# torque applied to the steering rack. It does not correlate to
# wheel slip, or to speed.

# This controller applies torque to achieve desired lateral
# accelerations. To compensate for the low speed effects we
# use a LOW_SPEED_FACTOR in the error. Additionally, there is
# friction in the steering wheel that needs to be overcome to
# move it at all, this is compensated for too.

LOW_SPEED_X = [0, 10, 20, 30]
LOW_SPEED_Y = [15, 13, 10, 5]

class LatControlTorque(LatControl):
  def __init__(self, CP, CI):
    super().__init__(CP, CI)
    self.torque_params = CP.lateralTuning.torque
    self.pid = PIDController(self.torque_params.kp, self.torque_params.ki,
                             k_f=self.torque_params.kf, pos_limit=self.steer_max, neg_limit=-self.steer_max)
    self.torque_from_lateral_accel = CI.torque_from_lateral_accel()
    self.use_steering_angle = self.torque_params.useSteeringAngle
    self.steering_angle_deadzone_deg = self.torque_params.steeringAngleDeadzoneDeg
    
    # neural network feedforward
    self.use_nn = CI.has_lat_torque_nnff
    if self.use_nn:
      cloudlog.warning("Using NNFF model for lateral torque control")
      self.torque_from_nn = CI.get_ff_nn
      self.error_downscale = 2.0 # downscale error in curves by up to the reciprocal of this factor
      self.error_downscale_deadzone = 0.3 # full error response on mostly straight roads
      # error downscaling uses planned lat accel to preemptively downscale error by 1.5s,
      # and is filtered so that downscaling continues for a short time after (~1.5s)
      # the curve ends.
      self.error_scale_factor = FirstOrderFilter(0.0, 0.5, 0.01) 
      # NNFF model takes current v_ego, lat_accel, lat accel/jerk error, roll, and past/future/planned data
      # of lat accel and roll
      # Past value is computed using previous desired lat accel and observed roll
      
      # setup future time offsets
      self.nnff_time_offset = CP.steerActuatorDelay + 0.2
      future_times = [0.3, 0.6, 1.0, 1.5] # seconds in the future
      self.nnff_future_times = [i + self.nnff_time_offset for i in future_times]
      # desired/planned lat accel is filtered to reduce noise
      self.nnff_lat_accels_filtered = [FirstOrderFilter(0.0, 0.0, 0.01) for i in [0.0] + future_times]
      self.nnff_alpha_up_down = [0.0, 0.0] # smoothing factors for increasing/decreasing magnitude of lat accel
      
      # setup past time offsets
      history_check_frames = [30, 20, 10] # 0.3, 0.2, 0.1 seconds ago
      self.history_frame_offsets = [history_check_frames[0] - i for i in history_check_frames]
      self.lat_accel_deque = deque(maxlen=history_check_frames[0])
      self.roll_deque = deque(maxlen=history_check_frames[0])

      # for scaling NNFF based on relative lat accel factor,
      # and NNFF error response based on relative friction factor
      self.live_NNFF_scaling = False # set to True to enable
      self.initial_lat_accel_factor = copy.copy(self.torque_params.latAccelFactor)
      self.initial_friction_coef = max(0.001, copy.copy(self.torque_params.friction))


  def update_live_torque_params(self, latAccelFactor, latAccelOffset, friction):
    self.torque_params.latAccelFactor = latAccelFactor
    self.torque_params.latAccelOffset = latAccelOffset
    self.torque_params.friction = friction

  def update(self, active, CS, VM, params, last_actuators, steer_limited, desired_curvature, desired_curvature_rate, llk, lat_plan=None, model_data=None):
    pid_log = log.ControlsState.LateralTorqueState.new_message()

    if not active:
      output_torque = 0.0
      pid_log.active = False
    else:
      if self.use_steering_angle:
        actual_curvature = -VM.calc_curvature(math.radians(CS.steeringAngleDeg - params.angleOffsetDeg), CS.vEgo, params.roll)
        curvature_deadzone = abs(VM.calc_curvature(math.radians(self.steering_angle_deadzone_deg), CS.vEgo, 0.0))
        if self.use_nn:
          actual_curvature_rate = -VM.calc_curvature(math.radians(CS.steeringRateDeg), CS.vEgo, 0.0)
          actual_lateral_jerk = actual_curvature_rate * CS.vEgo ** 2
      else:
        actual_curvature_vm = -VM.calc_curvature(math.radians(CS.steeringAngleDeg - params.angleOffsetDeg), CS.vEgo, params.roll)
        actual_curvature_llk = llk.angularVelocityCalibrated.value[2] / CS.vEgo
        actual_curvature = interp(CS.vEgo, [2.0, 5.0], [actual_curvature_vm, actual_curvature_llk])
        curvature_deadzone = 0.0
      desired_lateral_accel = desired_curvature * CS.vEgo ** 2

      # desired rate is the desired rate of change in the setpoint, not the absolute desired curvature
      # desired_lateral_jerk = desired_curvature_rate * CS.vEgo ** 2
      actual_lateral_accel = actual_curvature * CS.vEgo ** 2
      lateral_accel_deadzone = curvature_deadzone * CS.vEgo ** 2
      
      low_speed_factor = interp(CS.vEgo, LOW_SPEED_X, LOW_SPEED_Y)**2
      setpoint = desired_lateral_accel + low_speed_factor * desired_curvature
      measurement = actual_lateral_accel + low_speed_factor * actual_curvature
      if self.use_nn:
        # prepare input data for NNFF model
        
        # prepare past roll and error
        roll = params.roll
        self.roll_deque.append(roll)
        past_rolls = [self.roll_deque[min(len(self.roll_deque)-1, i)] for i in self.history_frame_offsets]
        error = desired_lateral_accel - actual_lateral_accel
        
        # prepare future roll, lat accel, and lat accel error
        if None not in [lat_plan, model_data] and all([len(i) >= CONTROL_N for i in [model_data.orientation.x, lat_plan.curvatures]]):
          adjusted_future_times = [t + 0.5*CS.aEgo*(t/max(CS.vEgo, 1.0)) for t in self.nnff_future_times]
          future_planned_lat_accels = [desired_lateral_accel] + [interp(t, T_IDXS[:CONTROL_N], lat_plan.curvatures) * CS.vEgo ** 2 for t in adjusted_future_times]
          future_rolls = [interp(t, T_IDXS, model_data.orientation.x) + roll for t in adjusted_future_times]
        else:
          future_planned_lat_accels = [desired_lateral_accel] * (len(self.nnff_future_times) + 1)
          future_rolls = [roll] * len(self.nnff_future_times)
          
        # filter future lat accel, save past lat accel
        alpha = self.nnff_alpha_up_down[0 if abs(desired_lateral_accel) > abs(self.nnff_lat_accels_filtered[0].x) else 1]
        for i,v in enumerate(future_planned_lat_accels):
          self.nnff_lat_accels_filtered[i].update_alpha(alpha)
          self.nnff_lat_accels_filtered[i].update(v)
        lat_accels_filtered = [i.x for i in self.nnff_lat_accels_filtered]
        self.lat_accel_deque.append(lat_accels_filtered[0])
        past_lateral_accels = [self.lat_accel_deque[min(len(self.lat_accel_deque)-1, i)] for i in self.history_frame_offsets]

        # compute friction factor and laf_ratio_stock_to_live
        if self.live_NNFF_scaling:
          friction_ratio_live_to_stock = self.torque_params.friction / self.initial_friction_coef
          laf_ratio_stock_to_live = self.initial_lat_accel_factor / max(0.5, self.torque_params.latAccelFactor)
        else:
          friction_ratio_live_to_stock = 1.0
          laf_ratio_stock_to_live = 1.0
          
        # compute NNFF error response
        # lat accel model input is downscaled based on lateral acceleration,
        # which happens preemptively based on future lateral accel as well.
        # This allows for full error correction on straights while letting
        # feedforward handle curves.
        max_future_abs_lat_accel = apply_deadzone(max(abs(i) for i in future_planned_lat_accels), self.error_downscale_deadzone)
        error_scale_factor = 1.0 / clip(self.error_downscale * max_future_abs_lat_accel + 1.0, 0.0, self.error_downscale)
        if error_scale_factor < self.error_scale_factor.x:
          self.error_scale_factor.x = error_scale_factor
        else:
          self.error_scale_factor.update(error_scale_factor)
        nnff_error_input = [CS.vEgo, error * self.error_scale_factor.x]
        pid_log.error = self.torque_from_nn(nnff_error_input) * laf_ratio_stock_to_live
        
        # compute NNFF feedforward
        nnff_input = [CS.vEgo, lat_accels_filtered[0], 0.0, roll] \
                    + past_lateral_accels + lat_accels_filtered[1:] \
                    + past_rolls + future_rolls
        ff_no_friction = self.torque_from_nn(nnff_input)
        nnff_input[2] = error
        if self.use_steering_angle:
          # most cars get a lateral jerk error response as well
          desired_lateral_jerk = desired_curvature_rate * CS.vEgo ** 2
          lateral_jerk_error = desired_lateral_jerk - actual_lateral_jerk
          nnff_input[2] += 0.1 * lateral_jerk_error
        ff_w_friction = self.torque_from_nn(nnff_input)
        
        # compute output feedforward
        friction = (ff_w_friction - ff_no_friction) * friction_ratio_live_to_stock
        ff = ff_no_friction * laf_ratio_stock_to_live + friction
        nnff_log = nnff_input + nnff_error_input + \
                    [max_future_abs_lat_accel, 
                     self.error_scale_factor.x, 
                     friction, 
                     friction_ratio_live_to_stock, 
                     laf_ratio_stock_to_live]
      else:
        gravity_adjusted_lateral_accel = desired_lateral_accel - params.roll * ACCELERATION_DUE_TO_GRAVITY
        torque_from_setpoint = self.torque_from_lateral_accel(setpoint, self.torque_params, setpoint,
                                                      lateral_accel_deadzone, friction_compensation=False)
        torque_from_measurement = self.torque_from_lateral_accel(measurement, self.torque_params, measurement,
                                                      lateral_accel_deadzone, friction_compensation=False)
        pid_log.error = torque_from_setpoint - torque_from_measurement
        ff = self.torque_from_lateral_accel(gravity_adjusted_lateral_accel, self.torque_params,
                                            desired_lateral_accel - actual_lateral_accel,
                                            lateral_accel_deadzone, friction_compensation=True)

      freeze_integrator = steer_limited or CS.steeringPressed or CS.vEgo < 5
      output_torque = self.pid.update(pid_log.error,
                                      feedforward=ff,
                                      speed=CS.vEgo,
                                      freeze_integrator=freeze_integrator)

      pid_log.active = True
      pid_log.p = self.pid.p
      pid_log.i = self.pid.i
      pid_log.d = self.pid.d
      pid_log.f = self.pid.f
      if self.use_nn:
        pid_log.nnffLog = nnff_log
      pid_log.output = -output_torque
      pid_log.actualLateralAccel = actual_lateral_accel
      pid_log.desiredLateralAccel = desired_lateral_accel
      pid_log.saturated = self._check_saturation(self.steer_max - abs(output_torque) < 1e-3, CS, steer_limited)

    # TODO left is positive in this convention
    return -output_torque, 0.0, pid_log
