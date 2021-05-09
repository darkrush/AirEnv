import copy
import numpy as np
from itertools import product


def random_reset(field_range, state_before_reset, R_safe, all_reset = False, retry_time = 40):
    agent_number = len(state_before_reset)
    state_list  = copy.deepcopy(state_before_reset)
    enable_list = [ all_reset|state.crash|state.reach for  state in state_list]
    enable_tmp = True in enable_list

    # list all crashed agent or list all agent if all_reset
    crash_idx_list = []
    for idx ,state in enumerate(state_list):
        if state.crash or all_reset: crash_idx_list.append(idx)

    if len(crash_idx_list)>0:
        # reset agent crash and movable flag
        for idx in crash_idx_list:
            state_list[idx].crash = False
            state_list[idx].movable = True
        for try_time in range(retry_time):
            # random replace the crashed agent
            for idx in crash_idx_list:
                state_list[idx].x = np.random.uniform(field_range[0],field_range[1])
                state_list[idx].y = np.random.uniform(field_range[2],field_range[3])
                state_list[idx].theta = np.random.uniform(0,3.1415926*2)
            

            # check whether the random state_list is no conflict
            no_conflict = True
            for idx_a,idx_b in product(range(agent_number),range(agent_number)):
                if idx_a == idx_b: continue
                state_a = state_list[idx_a]
                state_b = state_list[idx_b]
                agent_dist = ((state_a.x-state_b.x)**2+(state_a.y-state_b.y)**2)**0.5
                no_conflict = agent_dist > 2*R_safe
                # retry if conflict
                if not no_conflict : break
            # stop retrying if no conflict
            if no_conflict: break
        # if not no_conflict: print('failed to place agent with no confiliction')

    # list all reached agent or list all agent if all_reset
    reach_idx_list = []
    for idx ,state in enumerate(state_list):
        if state.reach or all_reset: reach_idx_list.append(idx)

    if len(reach_idx_list)>0:
        for idx in reach_idx_list:
            state_list[idx].reach = False
            state_list[idx].movable = True
        for try_time in range(retry_time):
            # random replace the targets of reached agent
            for idx in reach_idx_list:
                state_list[idx].target_x = np.random.uniform(field_range[0], field_range[1])
                state_list[idx].target_y = np.random.uniform(field_range[2], field_range[3])
                
            # check whether the random state_list is no conflict
            no_conflict = True
            for idx_a,idx_b in product(range(agent_number),range(agent_number)):
                if idx_a == idx_b: continue
                state_a = state_list[idx_a]
                state_b = state_list[idx_b]
                agent_dist = ((state_a.target_x-state_b.target_x)**2+(state_a.target_y-state_b.target_y)**2)**0.5
                no_conflict = agent_dist > 2*R_safe
                # retry if conflict
                if not no_conflict : break
            # stop retrying if no conflict
            if no_conflict: break
        #if not no_conflict: print('failed to place target with no confiliction')
    return state_list, enable_list, enable_tmp
