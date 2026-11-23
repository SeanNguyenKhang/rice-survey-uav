"""Shared site constants for the X450 rice survey (used by the tools/ scripts).

World frame = ENU (x east, y north), origin = home point (worlds/rice.sdf).
Home = take-off spot at the NW corner of the paddy, by the North bank
(10.5792937, 106.4161294; picked on the QGC map 2026-11-10, Thu Thua, Long An).

Paddy edges were measured on a Google satellite screenshot (2026-11-10),
registered on the user's SW / SE corners and point 2 (residuals <= 1.7 m):
  West bank (along the N-S road)  bearing 346.31 deg, through CORNER_NW
  North bank                      bearing  88.35 deg, through CORNER_NW
  East bank                       >= 48.8 m east of the West bank (NE corner)

Survey frame (origin = CORNER_NW):
  +v = along the West bank to the NNW, +u = perpendicular to the ENE (into the paddy)
Lawn-mower lines run along the West bank: line k at u = INSET_WEST + k * spacing,
north end MARGIN_NORTH inside the North bank, LINE_LENGTH long (to the south);
lines must stay MARGIN_EAST inside the East bank.
"""
import math

HOME_LAT = 10.5792937     # deg, = PX4_HOME_LAT and <spherical_coordinates>
HOME_LON = 106.4161294    # deg

CORNER_NW = (-0.57, -5.50)       # paddy NW corner in world ENU [m] (from home)
BANK_BEARING_DEG = 346.31        # West bank / survey lines (pointing north)
NORTH_BANK_BEARING_DEG = 88.35   # North bank (pointing east)
EAST_BANK_MIN_U = 48.8           # closest East bank, metres east of the West bank

# South corners as sent by the user (Google Maps, 0.1" rounding = +-1.5 m)
PADDY_SW = (10.5786389, 106.4162778)   # 10°34'43.1"N 106°24'58.6"E
PADDY_SE = (10.5786667, 106.4167500)   # 10°34'43.2"N 106°25'00.3"E

INSET_WEST = 3.0          # line 1 this far inside the West bank [m]
MARGIN_NORTH = 3.0        # north ends this far inside the North bank [m]
MARGIN_EAST = 3.0         # lines at least this far inside the East bank [m]
LINE_LENGTH = 50.0        # every survey line [m]

YAW = math.radians(360.0 - BANK_BEARING_DEG)        # ENU yaw of the survey frame (CCW)
_NORTH_SLOPE = math.tan(math.radians(NORTH_BANK_BEARING_DEG - (BANK_BEARING_DEG + 90.0 - 360.0)))

_M_PER_DEG_LAT = 111320.0
_M_PER_DEG_LON = 111320.0 * math.cos(math.radians(HOME_LAT))


def enu_to_latlon(x, y):
    """Local ENU metres -> (lat, lon) degrees. Flat-earth, fine for < 1 km."""
    return HOME_LAT + y / _M_PER_DEG_LAT, HOME_LON + x / _M_PER_DEG_LON


def latlon_to_enu(lat, lon):
    return (lon - HOME_LON) * _M_PER_DEG_LON, (lat - HOME_LAT) * _M_PER_DEG_LAT


def survey_to_enu(u, v):
    """Survey frame (u east of the West bank, v north along it) -> world ENU."""
    c, s = math.cos(YAW), math.sin(YAW)
    return CORNER_NW[0] + c * u - s * v, CORNER_NW[1] + s * u + c * v


def enu_to_survey(x, y):
    """World ENU -> survey frame (u, v)."""
    c, s = math.cos(YAW), math.sin(YAW)
    dx, dy = x - CORNER_NW[0], y - CORNER_NW[1]
    return c * dx + s * dy, -s * dx + c * dy


def north_bank_v(u):
    """v of the North bank at lateral position u (the bank drops to the south going east)."""
    return -u * _NORTH_SLOPE


def paddy_corners_uv():
    """Paddy corners NW, NE, SE, SW in the survey frame (East bank straight NE -> SE)."""
    se = enu_to_survey(*latlon_to_enu(*PADDY_SE))
    sw = enu_to_survey(*latlon_to_enu(*PADDY_SW))
    return [(0.0, 0.0), (EAST_BANK_MIN_U, north_bank_v(EAST_BANK_MIN_U)), se, (0.0, sw[1])]
