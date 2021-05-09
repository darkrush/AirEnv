from src.AirEnv import MultiCarSim
from src.paser import parse_senario
import random
import time
sd = parse_senario('src/scenarios/4p2.yaml')
Env = MultiCarSim(sd,sim_step = 10)
for j in range (10):
    Env.reset()
    for i in range(10):
        Env.render()
        action = []
        for i in range(6):
            action.append(random.uniform(-1,1))
        o,r,d,i = Env.step(action)
        print(i['crashed'])
        time.sleep(0)
