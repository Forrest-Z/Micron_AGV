bool health_ok

string[] topic_list # list of topics, in order.
float32[] health_points_list # list of health_points, in order.
string[] error_list # list of error messages, in order.

int32[] index_fail # list of index corresponding to the topics that failed health monitor's check.

string error_string #for shuo