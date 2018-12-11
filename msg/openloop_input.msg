# Set values to NAN if no openloop input is desired.

float32 thrust     # thrust input
float32 roll        # roll input
float32 pitch       # pitch input
float32 yaw         # yaw input

bool openloop_thrust   # should thrust be commanded
bool openloop_roll      # should roll be commanded
bool openloop_pitch     # should pitch be commanded
bool openloop_yaw       # should yaw be commanded