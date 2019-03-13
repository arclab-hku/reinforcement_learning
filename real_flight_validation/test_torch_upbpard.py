from sac import *
load_all_models(False)
import numpy as np
import time
a=time.time()
state=np.random.randn(1,10)
for i in range(1000):
    action = policy_net.get_action(np.array(state))

b=time.time()
print(b-a)
# this costs about 11 seconds...
# can potentailly run less than 100 hz.. so 50hz should be okay..
