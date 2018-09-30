# Set values to NAN if no step input is desired.

bool valid			# is the step input valid

float32 x			# NORTH position in the NED axis in m
float32 y			# EAST position in the NED axis in m
float32 z			# DOWN position in the NED axis in m

float32 vx			# NORTH velocity in the NED axis in m/s
float32 vy			# EAST velocity in the NED axis in m/s
float32 vz			# DOWN velocity in the NED axis in m/s

float32 roll		# roll angle in rad
float32 pitch		# pitch angle in rad
float32 yaw			# yaw angle in rad

float32 rollspeed	# roll angular rate in rad/s
float32 pitchspeed	# pitch angular rate in rad/s
float32 yawspeed	# yaw angular speed in rad/s

bool control_roll	# should roll be controlled
bool control_pitch	# should pitch be controlled
bool control_yaw	# should yaw be controlled