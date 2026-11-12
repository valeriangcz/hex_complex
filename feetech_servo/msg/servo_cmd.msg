# Global override: false disables all six servos and ignores angles.
bool enable

# Per-leg torque enable. Effective enable = enable AND enable_<leg>.
bool enable_lf
bool enable_lm
bool enable_lb
bool enable_rf
bool enable_rm
bool enable_rb

# Target angles in degrees relative to each configured mechanical zero.
# Absolute angle = offset_deg + direction * target angle.
float64 lf
float64 lm
float64 lb
float64 rf
float64 rm
float64 rb
