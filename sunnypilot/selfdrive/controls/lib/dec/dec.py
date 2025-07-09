<<<<<<< HEAD
# The MIT License
#
# Copyright (c) 2019-, Rick Lan, dragonpilot community, and a number of other of contributors.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
#
# Version = 2025-1-18

import numpy as np
=======
"""
Copyright (c) 2021-, rav4kumar, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
# Version = 2025-6-30
>>>>>>> sunnypilot/dev-c3-new

from cereal import messaging
from opendbc.car import structs
from numpy import interp
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
<<<<<<< HEAD
from openpilot.sunnypilot.selfdrive.controls.lib.dec.constants import WMACConstants, SNG_State

# d-e2e, from modeldata.h
TRAJECTORY_SIZE = 33

HIGHWAY_CRUISE_KPH = 70

STOP_AND_GO_FRAME = 60

SET_MODE_TIMEOUT = 10

V_ACC_MIN = 9.72


class GenericMovingAverageCalculator:
  def __init__(self, window_size):
    self.window_size = window_size
    self.data = []
    self.total = 0

  def add_data(self, value: float) -> None:
    if len(self.data) == self.window_size:
      self.total -= self.data.pop(0)
    self.data.append(value)
    self.total += value

  def get_moving_average(self) -> float | None:
    return None if len(self.data) == 0 else self.total / len(self.data)

  def reset_data(self) -> None:
    self.data = []
    self.total = 0


class WeightedMovingAverageCalculator:
  def __init__(self, window_size):
    self.window_size = window_size
    self.data = []
    self.weights = np.linspace(1, 3, window_size)  # Linear weights, adjust as needed

  def add_data(self, value: float) -> None:
    if len(self.data) == self.window_size:
      self.data.pop(0)
    self.data.append(value)

  def get_weighted_average(self) -> float | None:
    if len(self.data) == 0:
      return None
    weighted_sum: float = float(np.dot(self.data, self.weights[-len(self.data):]))
    weight_total: float = float(np.sum(self.weights[-len(self.data):]))
    return weighted_sum / weight_total

  def reset_data(self) -> None:
    self.data = []
=======
from openpilot.sunnypilot.selfdrive.controls.lib.dec.constants import WMACConstants
from typing import Literal

# d-e2e, from modeldata.h
TRAJECTORY_SIZE = 33
SET_MODE_TIMEOUT = 15

# Define the valid mode types
ModeType = Literal['acc', 'blended']


class SmoothKalmanFilter:
  """Enhanced Kalman filter with smoothing for stable decision making."""

  def __init__(self, initial_value=0, measurement_noise=0.1, process_noise=0.01,
               alpha=1.0, smoothing_factor=0.85):
    self.x = initial_value
    self.P = 1.0
    self.R = measurement_noise
    self.Q = process_noise
    self.alpha = alpha
    self.smoothing_factor = smoothing_factor
    self.initialized = False
    self.history = []
    self.max_history = 10
    self.confidence = 0.0

  def add_data(self, measurement):
    if len(self.history) >= self.max_history:
      self.history.pop(0)
    self.history.append(measurement)

    if not self.initialized:
      self.x = measurement
      self.initialized = True
      self.confidence = 0.1
      return

    self.P = self.alpha * self.P + self.Q

    K = self.P / (self.P + self.R)
    effective_K = K * (1.0 - self.smoothing_factor) + self.smoothing_factor * 0.1

    innovation = measurement - self.x
    self.x = self.x + effective_K * innovation
    self.P = (1 - effective_K) * self.P

    if abs(innovation) < 0.1:
      self.confidence = min(1.0, self.confidence + 0.05)
    else:
      self.confidence = max(0.1, self.confidence - 0.02)

  def get_value(self):
    return self.x if self.initialized else None

  def get_confidence(self):
    return self.confidence

  def reset_data(self):
    self.initialized = False
    self.history = []
    self.confidence = 0.0


class ModeTransitionManager:
  """Manages smooth transitions between driving modes with hysteresis."""

  def __init__(self):
    self.current_mode: ModeType = 'acc'
    self.mode_confidence = {'acc': 1.0, 'blended': 0.0}
    self.transition_timeout = 0
    self.min_mode_duration = 10
    self.mode_duration = 0
    self.emergency_override = False

  def request_mode(self, mode: ModeType, confidence: float = 1.0, emergency: bool = False):
    # Emergency override for critical situations (stops, collisions)
    if emergency:
      self.emergency_override = True
      self.current_mode = mode
      self.transition_timeout = SET_MODE_TIMEOUT
      self.mode_duration = 0
      return

    self.mode_confidence[mode] = min(1.0, self.mode_confidence[mode] + 0.1 * confidence)
    for m in self.mode_confidence:
      if m != mode:
        self.mode_confidence[m] = max(0.0, self.mode_confidence[m] - 0.05)

    # Require minimum duration in current mode (unless emergency)
    if self.mode_duration < self.min_mode_duration and not self.emergency_override:
      return

    # Hysteresis: higher threshold for mode changes
    confidence_threshold = 0.6 if mode != self.current_mode else 0.3  # Lower threshold for faster response

    if self.mode_confidence[mode] > confidence_threshold:
      if mode != self.current_mode and self.transition_timeout == 0:
        self.transition_timeout = SET_MODE_TIMEOUT
        self.current_mode = mode
        self.mode_duration = 0

  def update(self):
    if self.transition_timeout > 0:
      self.transition_timeout -= 1
    self.mode_duration += 1

    # Reset emergency override after some time
    if self.emergency_override and self.mode_duration > 20:
      self.emergency_override = False

    # Gradual confidence decay
    for mode in self.mode_confidence:
      self.mode_confidence[mode] *= 0.98

  def get_mode(self) -> ModeType:
    return self.current_mode
>>>>>>> sunnypilot/dev-c3-new


class DynamicExperimentalController:
  def __init__(self, CP: structs.CarParams, mpc, params=None):
    self._CP = CP
    self._mpc = mpc
    self._params = params or Params()
    self._enabled: bool = self._params.get_bool("DynamicExperimentalControl")
    self._active: bool = False
<<<<<<< HEAD
    self._mode: str = 'acc'
    self._frame: int = 0

    # Use weighted moving average for filtering leads
    self._lead_gmac = WeightedMovingAverageCalculator(window_size=WMACConstants.LEAD_WINDOW_SIZE)
    self._has_lead_filtered = False
    self._has_lead_filtered_prev = False

    self._slow_down_gmac = WeightedMovingAverageCalculator(window_size=WMACConstants.SLOW_DOWN_WINDOW_SIZE)
    self._has_slow_down: bool = False
    self._slow_down_confidence: float = 0.0

    self._has_blinkers = False

    self._slowness_gmac = WeightedMovingAverageCalculator(window_size=WMACConstants.SLOWNESS_WINDOW_SIZE)
    self._has_slowness: bool = False

    self._has_nav_instruction = False

    self._dangerous_ttc_gmac = WeightedMovingAverageCalculator(window_size=WMACConstants.DANGEROUS_TTC_WINDOW_SIZE)
    self._has_dangerous_ttc: bool = False

    self._v_ego_kph = 0.
    self._v_cruise_kph = 0.

    self._has_lead = False

    self._has_standstill = False
    self._has_standstill_prev = False

    self._sng_transit_frame = 0
    self._sng_state = SNG_State.off

    self._mpc_fcw_gmac = WeightedMovingAverageCalculator(window_size=WMACConstants.MPC_FCW_WINDOW_SIZE)
    self._has_mpc_fcw: bool = False
    self._mpc_fcw_crash_cnt = 0

    self._set_mode_timeout = 0
=======
    self._frame: int = 0
    self._urgency = 0.0

    self._mode_manager = ModeTransitionManager()

    # Smooth filters for stable decision making with faster response for critical scenarios
    self._lead_filter = SmoothKalmanFilter(
      measurement_noise=0.15,
      process_noise=0.05,
      alpha=1.02,
      smoothing_factor=0.8
    )

    self._slow_down_filter = SmoothKalmanFilter(
      measurement_noise=0.1,
      process_noise=0.1,
      alpha=1.05,
      smoothing_factor=0.7
    )

    self._slowness_filter = SmoothKalmanFilter(
      measurement_noise=0.1,
      process_noise=0.06,
      alpha=1.015,
      smoothing_factor=0.92
    )

    self._mpc_fcw_filter = SmoothKalmanFilter(
      measurement_noise=0.2,
      process_noise=0.1,
      alpha=1.1,
      smoothing_factor=0.5
    )
    self._has_lead_filtered = False
    self._has_slow_down = False
    self._has_slowness = False
    self._has_mpc_fcw = False
    self._v_ego_kph = 0.0
    self._v_cruise_kph = 0.0
    self._has_standstill = False
    self._mpc_fcw_crash_cnt = 0
    self._standstill_count = 0
    # debug
    self._endpoint_x = float('inf')
    self._expected_distance = 0.0
    self._trajectory_valid = False
>>>>>>> sunnypilot/dev-c3-new

  def _read_params(self) -> None:
    if self._frame % int(1. / DT_MDL) == 0:
      self._enabled = self._params.get_bool("DynamicExperimentalControl")

  def mode(self) -> str:
<<<<<<< HEAD
    return str(self._mode)
=======
    return self._mode_manager.get_mode()
>>>>>>> sunnypilot/dev-c3-new

  def enabled(self) -> bool:
    return self._enabled

  def active(self) -> bool:
    return self._active

<<<<<<< HEAD
  @staticmethod
  def _anomaly_detection(recent_data: list[float], threshold: float = 2.0, context_check: bool = True) -> bool:
    """
    Basic anomaly detection using standard deviation.
    """
    if len(recent_data) < 5:
      return False
    mean: float = float(np.mean(recent_data))
    std_dev: float = float(np.std(recent_data))
    anomaly: bool = bool(recent_data[-1] > mean + threshold * std_dev)

    # Context check to ensure repeated anomaly
    if context_check:
      return np.count_nonzero(np.array(recent_data) > mean + threshold * std_dev) > 1
    return anomaly

  def _adaptive_slowdown_threshold(self) -> float:
    """
    Adapts the slow-down threshold based on vehicle speed and recent behavior.
    """
    slowdown_scaling_factor: float = (1.0 + 0.03 * np.log(1 + len(self._slow_down_gmac.data)))
    adaptive_threshold: float = float(
      interp(self._v_ego_kph, WMACConstants.SLOW_DOWN_BP, WMACConstants.SLOW_DOWN_DIST) * slowdown_scaling_factor
    )
    return adaptive_threshold

  def _smoothed_lead_detection(self, lead_prob: float, smoothing_factor: float = 0.2):
    """
    Smoothing the lead detection to avoid erratic behavior.
    """
    lead_filtering: float = (1 - smoothing_factor) * self._has_lead_filtered + smoothing_factor * lead_prob
    return lead_filtering > WMACConstants.LEAD_PROB

  def _adaptive_lead_prob_threshold(self) -> float:
    """
    Adapts lead probability threshold based on driving conditions.
    """
    if self._v_ego_kph > HIGHWAY_CRUISE_KPH:
      return float(WMACConstants.LEAD_PROB + 0.1)  # Increase the threshold on highways
    return float(WMACConstants.LEAD_PROB)
=======
  def set_mpc_fcw_crash_cnt(self) -> None:
    """Set MPC FCW crash count"""
    self._mpc_fcw_crash_cnt = self._mpc.crash_cnt
>>>>>>> sunnypilot/dev-c3-new

  def _update_calculations(self, sm: messaging.SubMaster) -> None:
    car_state = sm['carState']
    lead_one = sm['radarState'].leadOne
    md = sm['modelV2']

    self._v_ego_kph = car_state.vEgo * 3.6
    self._v_cruise_kph = car_state.vCruise
<<<<<<< HEAD
    self._has_lead = lead_one.status
    self._has_standstill = car_state.standstill

    # fcw detection
    self._mpc_fcw_gmac.add_data(self._mpc_fcw_crash_cnt > 0)
    if _mpc_fcw_weighted_average := self._mpc_fcw_gmac.get_weighted_average():
      self._has_mpc_fcw = _mpc_fcw_weighted_average > WMACConstants.MPC_FCW_PROB
    else:
      self._has_mpc_fcw = False

    # nav enable detection
    # self._has_nav_instruction = md.navEnabledDEPRECATED and maneuver_distance / max(car_state.vEgo, 1) < 13

    # lead detection with smoothing
    self._lead_gmac.add_data(lead_one.status)
    self._has_lead_filtered = (self._lead_gmac.get_weighted_average() or -1.) > WMACConstants.LEAD_PROB
    #lead_prob = self._lead_gmac.get_weighted_average() or 0
    #self._has_lead_filtered = self._smoothed_lead_detection(lead_prob)

    # adaptive slow down detection
    adaptive_threshold = self._adaptive_slowdown_threshold()
    slow_down_trigger = len(md.orientation.x) == len(md.position.x) == TRAJECTORY_SIZE and md.position.x[TRAJECTORY_SIZE - 1] < adaptive_threshold
    self._slow_down_gmac.add_data(slow_down_trigger)
    if _has_slow_down_weighted_average := self._slow_down_gmac.get_weighted_average():
      self._has_slow_down = _has_slow_down_weighted_average > WMACConstants.SLOW_DOWN_PROB
      self._slow_down_confidence = _has_slow_down_weighted_average  # Store confidence level
    else:
      self._has_slow_down = False
      self._slow_down_confidence = 0.0  # No confidence if no slowdown

    # anomaly detection for slow down events
    if self._anomaly_detection(self._slow_down_gmac.data):
      self._slow_down_confidence *= 0.85  # Reduce confidence
      self._has_slow_down = self._slow_down_confidence > WMACConstants.SLOW_DOWN_PROB

    # blinker detection
    self._has_blinkers = car_state.leftBlinker or car_state.rightBlinker

    # sng detection
    if self._has_standstill:
      self._sng_state = SNG_State.stopped
      self._sng_transit_frame = 0
    else:
      if self._sng_transit_frame == 0:
        if self._sng_state == SNG_State.stopped:
          self._sng_state = SNG_State.going
          self._sng_transit_frame = STOP_AND_GO_FRAME
        elif self._sng_state == SNG_State.going:
          self._sng_state = SNG_State.off
      elif self._sng_transit_frame > 0:
        self._sng_transit_frame -= 1

    # slowness detection
    if not self._has_standstill:
      self._slowness_gmac.add_data(self._v_ego_kph <= (self._v_cruise_kph * WMACConstants.SLOWNESS_CRUISE_OFFSET))
      if _slowness_weighted_average := self._slowness_gmac.get_weighted_average():
        self._has_slowness = _slowness_weighted_average > WMACConstants.SLOWNESS_PROB
      else:
        self._has_slowness = False

    # dangerous TTC detection
    if not self._has_lead_filtered and self._has_lead_filtered_prev:
      self._dangerous_ttc_gmac.reset_data()
      self._has_dangerous_ttc = False

    if self._has_lead and car_state.vEgo >= 0.01:
      self._dangerous_ttc_gmac.add_data(lead_one.dRel / car_state.vEgo)

    if _dangerous_ttc_weighted_average := self._dangerous_ttc_gmac.get_weighted_average():
      self._has_dangerous_ttc = _dangerous_ttc_weighted_average <= WMACConstants.DANGEROUS_TTC
    else:
      self._has_dangerous_ttc = False

    # keep prev values
    self._has_standstill_prev = self._has_standstill
    self._has_lead_filtered_prev = self._has_lead_filtered

  def _radarless_mode(self) -> None:
    # when mpc fcw crash prob is high
    # use blended to slow down quickly
    if self._has_mpc_fcw:
      self._set_mode('blended')
      return

    # Nav enabled and distance to upcoming turning is 300 or below
    # if self._has_nav_instruction:
    #  self._set_mode('blended')
    #  return

    # when blinker is on and speed is driving below V_ACC_MIN: blended
    # we don't want it to switch mode at higher speed, blended may trigger hard brake
    # if self._has_blinkers and self._v_ego_kph < V_ACC_MIN:
    #  self._set_mode('blended')
    #  return

    # when at highway cruise and SNG: blended
    # ensuring blended mode is used because acc is bad at catching SNG lead car
    # especially those who accel very fast and then brake very hard.
    # if self._sng_state == SNG_State.going and self._v_cruise_kph >= V_ACC_MIN:
    #  self._set_mode('blended')
    #  return

    # when standstill: blended
    # in case of lead car suddenly move away under traffic light, acc mode won't brake at traffic light.
    if self._has_standstill:
      self._set_mode('blended')
      return

    # when detecting slow down scenario: blended
    # e.g. traffic light, curve, stop sign etc.
    if self._has_slow_down:
      self._set_mode('blended')
      return

    # when detecting lead slow down: blended
    # use blended for higher braking capability
    if self._has_dangerous_ttc:
      self._set_mode('blended')
      return

    # car driving at speed lower than set speed: acc
    if self._has_slowness:
      self._set_mode('acc')
      return

    self._set_mode('acc')

  def _radar_mode(self) -> None:
    # when mpc fcw crash prob is high
    # use blended to slow down quickly
    if self._has_mpc_fcw:
      self._set_mode('blended')
      return

    # If there is a filtered lead, the vehicle is not in standstill, and the lead vehicle's yRel meets the condition,
    if self._has_lead_filtered and not self._has_standstill:
      self._set_mode('acc')
      return

    # when blinker is on and speed is driving below V_ACC_MIN: blended
    # we don't want it to switch mode at higher speed, blended may trigger hard brake
    # if self._has_blinkers and self._v_ego_kph < V_ACC_MIN:
    #  self._set_mode('blended')
    #  return

    # when standstill: blended
    # in case of lead car suddenly move away under traffic light, acc mode won't brake at traffic light.
    if self._has_standstill:
      self._set_mode('blended')
      return

    # when detecting slow down scenario: blended
    # e.g. traffic light, curve, stop sign etc.
    if self._has_slow_down:
      self._set_mode('blended')
      return

    # car driving at speed lower than set speed: acc
    if self._has_slowness:
      self._set_mode('acc')
      return

    # Nav enabled and distance to upcoming turning is 300 or below
    # if self._has_nav_instruction:
    #  self._set_mode('blended')
    #  return

    self._set_mode('acc')

  def set_mpc_fcw_crash_cnt(self) -> None:
    self._mpc_fcw_crash_cnt = self._mpc.crash_cnt

  def _set_mode(self, mode: str) -> None:
    if self._set_mode_timeout == 0:
      self._mode = mode
      if mode == 'blended':
        self._set_mode_timeout = SET_MODE_TIMEOUT

    if self._set_mode_timeout > 0:
      self._set_mode_timeout -= 1
=======
    self._has_standstill = car_state.standstill

    # standstill detection
    if self._has_standstill:
      self._standstill_count = min(20, self._standstill_count + 1)
    else:
      self._standstill_count = max(0, self._standstill_count - 1)

    # Lead detection
    self._lead_filter.add_data(float(lead_one.status))
    lead_value = self._lead_filter.get_value() or 0.0
    self._has_lead_filtered = lead_value > WMACConstants.LEAD_PROB

    # MPC FCW detection
    fcw_filtered_value = self._mpc_fcw_filter.get_value() or 0.0
    self._mpc_fcw_filter.add_data(float(self._mpc_fcw_crash_cnt > 0))
    self._has_mpc_fcw = fcw_filtered_value > 0.5

    # Slow down detection
    self._calculate_slow_down(md)

    # Slowness detection
    if not (self._standstill_count > 5) and not self._has_slow_down:
      current_slowness = float(self._v_ego_kph <= (self._v_cruise_kph * WMACConstants.SLOWNESS_CRUISE_OFFSET))
      self._slowness_filter.add_data(current_slowness)
      slowness_value = self._slowness_filter.get_value() or 0.0

      # Hysteresis for slowness
      threshold = WMACConstants.SLOWNESS_PROB * (0.8 if self._has_slowness else 1.1)
      self._has_slowness = slowness_value > threshold

  def _calculate_slow_down(self, md):
    """Calculate urgency based on trajectory endpoint vs expected distance."""

    # Reset to safe defaults
    urgency = 0.0
    self._endpoint_x = float('inf')
    self._trajectory_valid = False

    #Require exact trajectory size
    position_valid = len(md.position.x) == TRAJECTORY_SIZE
    orientation_valid = len(md.orientation.x) == TRAJECTORY_SIZE

    if not (position_valid and orientation_valid):
      # Invalid trajectory - this itself might indicate a stop scenario
      # Apply moderate urgency for incomplete trajectories at speed
      if self._v_ego_kph > 20.0:
        urgency = 0.3

      self._slow_down_filter.add_data(urgency)
      urgency_filtered = self._slow_down_filter.get_value() or 0.0
      self._has_slow_down = urgency_filtered > WMACConstants.SLOW_DOWN_PROB
      self._urgency = urgency_filtered
      return

    # We have a valid full trajectory
    self._trajectory_valid = True

    # Use the exact endpoint (33rd point, index 32)
    endpoint_x = md.position.x[TRAJECTORY_SIZE - 1]
    self._endpoint_x = endpoint_x

    # Get expected distance based on current speed using tuned constants
    expected_distance = interp(self._v_ego_kph,
                               WMACConstants.SLOW_DOWN_BP,
                               WMACConstants.SLOW_DOWN_DIST)
    self._expected_distance = expected_distance

    # Calculate urgency based on trajectory shortage
    if endpoint_x < expected_distance:
      shortage = expected_distance - endpoint_x
      shortage_ratio = shortage / expected_distance

      # Base urgency on shortage ratio
      urgency = min(1.0, shortage_ratio * 2.0)

      # Increase urgency for very short trajectories (imminent stops)
      critical_distance = expected_distance * 0.3
      if endpoint_x < critical_distance:
        urgency = min(1.0, urgency * 2.0)

      # Speed-based urgency adjustment
      if self._v_ego_kph > 25.0:
        speed_factor = 1.0 + (self._v_ego_kph - 25.0) / 80.0
        urgency = min(1.0, urgency * speed_factor)

    # Apply filtering but with less smoothing for stops
    self._slow_down_filter.add_data(urgency)
    urgency_filtered = self._slow_down_filter.get_value() or 0.0

    # Update state with lower threshold for better stop detection
    self._has_slow_down = urgency_filtered > (WMACConstants.SLOW_DOWN_PROB * 0.8)
    self._urgency = urgency_filtered

  def _radarless_mode(self) -> None:
    """Radarless mode decision logic with emergency handling."""

    # EMERGENCY: MPC FCW - immediate blended mode
    if self._has_mpc_fcw:
      self._mode_manager.request_mode('blended', confidence=1.0, emergency=True)
      return

    # Standstill: use blended
    if self._standstill_count > 3:
      self._mode_manager.request_mode('blended', confidence=0.9)
      return

    # Slow down scenarios: emergency for high urgency, normal for lower urgency
    if self._has_slow_down:
      if self._urgency > 0.7:
        # Emergency: immediate blended mode for high urgency stops
        self._mode_manager.request_mode('blended', confidence=1.0, emergency=True)
      else:
        # Normal: blended with urgency-based confidence
        confidence = min(1.0, self._urgency * 1.5)
        self._mode_manager.request_mode('blended', confidence=confidence)
      return

    # Driving slow: use ACC (but not if actively slowing down)
    if self._has_slowness and not self._has_slow_down:
      self._mode_manager.request_mode('acc', confidence=0.8)
      return

    # Default: ACC
    self._mode_manager.request_mode('acc', confidence=0.7)

  def _radar_mode(self) -> None:
    """Radar mode with emergency handling."""

    # EMERGENCY: MPC FCW - immediate blended mode
    if self._has_mpc_fcw:
      self._mode_manager.request_mode('blended', confidence=1.0, emergency=True)
      return

    # If lead detected and not in standstill: always use ACC
    if self._has_lead_filtered and not (self._standstill_count > 3):
      self._mode_manager.request_mode('acc', confidence=1.0)
      return

    # Slow down scenarios: emergency for high urgency, normal for lower urgency
    if self._has_slow_down:
      if self._urgency > 0.7:
        # Emergency: immediate blended mode for high urgency stops
        self._mode_manager.request_mode('blended', confidence=1.0, emergency=True)
      else:
        # Normal: blended with urgency-based confidence
        confidence = min(1.0, self._urgency * 1.3)
        self._mode_manager.request_mode('blended', confidence=confidence)
      return

    # Standstill: use blended
    if self._standstill_count > 3:
      self._mode_manager.request_mode('blended', confidence=0.9)
      return

    # Driving slow: use ACC (but not if actively slowing down)
    if self._has_slowness and not self._has_slow_down:
      self._mode_manager.request_mode('acc', confidence=0.8)
      return

    # Default: ACC
    self._mode_manager.request_mode('acc', confidence=0.7)
>>>>>>> sunnypilot/dev-c3-new

  def update(self, sm: messaging.SubMaster) -> None:
    self._read_params()

    self.set_mpc_fcw_crash_cnt()

    self._update_calculations(sm)

    if self._CP.radarUnavailable:
      self._radarless_mode()
    else:
      self._radar_mode()

<<<<<<< HEAD
    self._active = sm['selfdriveState'].experimentalMode and self._enabled

=======
    self._mode_manager.update()
    self._active = sm['selfdriveState'].experimentalMode and self._enabled
>>>>>>> sunnypilot/dev-c3-new
    self._frame += 1
