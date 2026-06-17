std_msgs/Header header
int32 id
bool is_agent
# conformal classification set: Mondrian class codes (0=HUMAN, 1=VEHICLE_LIKE, 2=OTHER).
# EMPTY -> sando_node falls back to the legacy id heuristic (id>=200 -> wall/soft, else human/hard).
int32[] label_set
float64[] bbox
string[] function
string[] velocity
geometry_msgs/Vector3 pos
PWPTraj pwp
# fields set by obstacle_tracker (EKF covariances + quintic poly prediction)
float64[] ekf_cov_p
float64[] ekf_cov_q
float64[] ekf_cov_r
float64[] poly_cov
float64[] poly_coeffs_x
float64[] poly_coeffs_y
float64[] poly_coeffs_z
float64 poly_start_time
float64 poly_end_time
