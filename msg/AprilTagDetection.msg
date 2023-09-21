int32[] id
float64[] size
float64 decision_margin
float64[2] center
float64[8] corners # [[float, float] * 4]
geometry_msgs/Point[4] corners3
float64[9] R # [3x3]
float64[3] t # [1x3]
float64 err
geometry_msgs/PoseWithCovarianceStamped pose