std_msgs/Header header
float64 batch_size
float64 app_flo
float64 execution_time
float64 required_flops

int64 id                        # Agent unique 
int64 CAV_flop                  # Available computer resources
int64 CAV_t_available           # Estimated Available Time in the network scope in miliseconds
int64 CAV_total_flop            # Distributed Network total available compute resource
bool IS_LEADER                  # If selected as leader to distributed computation
RAD_VEGA_POPULATION P           # population vector with size of N_p*S_size
