class WMACConstants:
<<<<<<< HEAD
  LEAD_WINDOW_SIZE = 5
  LEAD_PROB = 0.5

  SLOW_DOWN_WINDOW_SIZE = 5
  SLOW_DOWN_PROB = 0.6

  SLOW_DOWN_BP = [0., 10., 20., 30., 40., 50., 55., 60.]
  SLOW_DOWN_DIST = [25., 38., 55., 75., 95., 115., 130., 150.]

  SLOWNESS_WINDOW_SIZE = 12
  SLOWNESS_PROB = 0.5
  SLOWNESS_CRUISE_OFFSET = 1.05

  DANGEROUS_TTC_WINDOW_SIZE = 3
  DANGEROUS_TTC = 2.3

  MPC_FCW_WINDOW_SIZE = 10
  MPC_FCW_PROB = 0.5


class SNG_State:
  off = 0
  stopped = 1
  going = 2
=======
  # Lead detection parameters
  LEAD_WINDOW_SIZE = 6  # Stable detection window
  LEAD_PROB = 0.45  # Balanced threshold for lead detection

  # Slow down detection parameters
  SLOW_DOWN_WINDOW_SIZE = 5  # Responsive but stable
  SLOW_DOWN_PROB = 0.3  # Balanced threshold for slow down scenarios

  # Optimized slow down distance curve - smooth and progressive
  SLOW_DOWN_BP = [0., 10., 20., 30., 40., 50., 55., 60.]
  SLOW_DOWN_DIST = [32., 46., 64., 86., 108., 130., 145., 165.]

  # Slowness detection parameters
  SLOWNESS_WINDOW_SIZE = 10  # Stable slowness detection
  SLOWNESS_PROB = 0.55  # Clear threshold for slowness
  SLOWNESS_CRUISE_OFFSET = 1.025  # Conservative cruise speed offset
>>>>>>> sunnypilot/dev-c3-new
