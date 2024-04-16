string DEVICEMODE_PRESS     = "pressMode"
string DEVICEMODE_SWITCH    = "switchMode"
string DEVICEMODE_CUSTOMIZE = "customizeMode"

Header  header       # timestamp

float64 battery      # the current battery level, 0-100

bool    power        # ON/OFF state True/False
string  device_mode  # pressMode, switchMode, or customizeMode
