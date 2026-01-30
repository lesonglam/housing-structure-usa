# config.py

# Which position channel to control from:
# "P1" uses IN1 (A1/AD1 on Arduino), "P2" uses IN2
POSITION_CHANNEL = "P1"

# Position limits (inches). Used for clamping targets and safety.
MIN_INCHES = 0.00
MAX_INCHES = 10.00

# Clamp raw ADC to calibrated range
# Measure these once by jogging to ends and reading live ADC
ADC_MIN = 0      # example: fully retracted
ADC_MAX =  460      # example: fully extended

# Physical stroke length (inches)
STROKE_INCHES = 10.0

# Clamp raw ADC to calibrated range
CLAMP_ADC = True

# Optional: invert sensor direction
# True if ADC decreases when actuator extends
INVERT_POSITION = False

# Control tolerance around target (inches)
TOL_INCHES = 0.1

# How often the GUI/control loop updates (ms)
GUI_POLL_MS = 50

# How often to re-send motion command while moving (ms)
# Helps robustness if a byte drops, and keeps actuator moving during long steps.
COMMAND_REPEAT_MS = 120

# Safety: if telemetry stops arriving for too long, force STOP (seconds)
TELEMETRY_TIMEOUT_S = 0.75

# Serial defaults
DEFAULT_BAUD = 9600
SERIAL_TIMEOUT_S = 0.5

# Optional: require "fresh" telemetry before starting
REQUIRE_TELEMETRY_BEFORE_START = True
