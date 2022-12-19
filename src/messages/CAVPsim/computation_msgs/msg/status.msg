std_msgs/Header header

int16 id                        # Agent unique 
int32 CAV_flop                  # Available computer resources
int32 CAV_t_available           # Estimated Available Time in the network scope in miliseconds
bool IS_LEADER                  # If selected as leader to distributed computation
RAD_VEGA_POPULATION P           # population vector with size of N_p*S_size
computation_msgs/NET_MEMBER[] net_member