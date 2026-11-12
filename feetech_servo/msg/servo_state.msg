# Timestamp at the start of this sequential read cycle, not simultaneous sampling.
std_msgs/Header header

# Measured angles in degrees, using the command's zero and direction.
# A failed read produces NaN and valid=false for that servo.
float64 lf
float64 lm
float64 lb
float64 rf
float64 rm
float64 rb

# lf, lm, lb, rf, rm, rb; valid means a usable position read, not target reached.
bool[6] valid
