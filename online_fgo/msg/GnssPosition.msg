# Extended NavSatFix message

Header header

# satellite fix status information
sensor_msgs/NavSatStatus status

# Latitude [degrees]. Positive is north of equator; negative is south.
float64 latitude

# Longitude [degrees]. Positive is east of prime meridian; negative is west.
float64 longitude

# Altitude [m]. Positive is above the WGS 84 ellipsoid
# (quiet NaN if no altitude is available).
float64 altitude

# Position covariance [m^2] defined relative to a tangential plane
# through the reported position. The components are East, North, and
# Up (ENU), in row-major order.
#
# Beware: this coordinate system exhibits singularities at the poles.

float64[9] position_covariance

bool have_speed
# speed ns, ew, ud
float64[3] speed
# vn(m/s), ve(m/s), vu(m/s), sdvn, sdve, sdvu, sdvne, sdveu, sdvun
float64[6] speed_cov

# If the covariance of the fix is known, fill it in completely. If the
# GPS receiver provides the variance of each measurement, put them
# along the diagonal. If only Dilution of Precision is available,
# estimate an approximate covariance from that.

uint8 COVARIANCE_TYPE_UNKNOWN = 0
uint8 COVARIANCE_TYPE_APPROXIMATED = 1
uint8 COVARIANCE_TYPE_DIAGONAL_KNOWN = 2
uint8 COVARIANCE_TYPE_KNOWN = 3

uint8 position_covariance_type

# ===== NavSatFix fields end =====

# Result of ratio test performed on the potential "Fix" solution,
# it shows how many times is the best solution better than the next one.
# If this number is more than 3 Reach will consider RTK solution Fixed.
float64 ar_validation_ratio
