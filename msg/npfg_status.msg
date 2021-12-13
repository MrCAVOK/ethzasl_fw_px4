uint64 timestamp                # time since system start (microseconds)

uint8 wind_est_valid            # (boolean) true = wind estimate is valid and/or being used by controller (also indicates if wind est usage is disabled despite being valid)
float32 lat_accel               # resultant lateral acceleration reference [m/s^2]
float32 lat_accel_ff            # lateral acceleration demand only for maintaining curvature [m/s^2]
float32 bearing_feas            # bearing feasibility [0,1]
float32 bearing_feas_on_track   # on-track bearing feasibility [0,1]
float32 signed_track_error      # signed track error [m]
float32 track_error_bound       # track error bound [m]
float32 airspeed_ref            # airspeed reference [m/s]
float32 bearing                 # bearing angle [rad]
float32 heading_ref             # heading angle reference [rad]
float32 min_ground_speed_ref    # minimum forward ground speed reference [m/s]
float32 adapted_period          # adapted period (if auto-tuning enabled) [s]
float32 p_gain                  # controller proportional gain [rad/s]
float32 time_const              # controller time constant [s]
uint8 segment_complete		# (boolean) true = segment is completed and hence last waypoint of the segment is passed
float32 unit_tan_x		# unit tangent x (North) component [m]
float32 unit_tan_y		# unit tangent y (East) component [m]
float32 path_curvature		# path curvature [1/m]
float32 nxt_wp_x		# next waypoint x (North) component [m]
float32 nxt_wp_y		# next waypoint y (East) component [m]
float32 wp_cnt			# number of current wp in segment [-]
