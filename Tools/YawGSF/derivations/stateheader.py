from common import *

print("#pragma once")

print('static const int N_STATES = %u;' % (n_states,))
print('static const int PACKED_COVARIANCE_SIZE = %u;' % (n_states*(n_states+1)//2,))

for line in state_idx_defines:
    print(line)