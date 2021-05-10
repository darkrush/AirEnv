import numpy
import time
import copy
import random

from .basic import AgentProp,AgentState,Fence

def random_reset(field_range, state_before_reset, fence_list):
    state_list = copy.deepcopy(state_before_reset)
    for idx in range(len(state_list)):
        while(True):
            well_placed = True
            x = random.uniform(field_range[0],field_range[1])
            y = random.uniform(field_range[2],field_range[3])
            for fence in fence_list:
                dist = ((fence.anchor[0]-x)**2 + (fence.anchor[1]-y)**2)**0.5
                if dist<fence.radius:
                    well_placed = False
            if well_placed:
                break
        state_list[idx].x = x
        state_list[idx].y = y
        state_list[idx].theta = random.uniform(0,2*3.1415)
    
    return state_list

class MultiCarSim(object):

    def __init__(self, senario_dict, step_t = 40, sim_step = 30):

        #self.global_agent_prop = AgentProp(senario_dict['default_agent'])

        self.time_limit = senario_dict['common']['time_limit']
        self.reward_coef = senario_dict['common']['reward_coef']
        self.field_range = senario_dict['common']['field_range']

        self.step_t = step_t
        self.sim_step = sim_step
        self.sim_t = step_t/sim_step

        # reset_mode is 'random' or 'init'
        self.reset_mode = senario_dict['common']['reset_mode']
        

        self.agent_number = 0
        self.enermy_number = 0
        for group in senario_dict['agent_groups'].values():
            for agent_prop in group:
                if agent_prop['R_B'] == 'red':
                    self.agent_number += 1
                elif agent_prop['R_B'] == 'blue':
                    self.enermy_number += 1
        self.fence_number = 0
        if 'fence_groups' in senario_dict.keys():
            for group in senario_dict['fence_groups'].values():
                for agent_prop in group:
                    self.fence_number += 1

        self.agent_prop_list = []
        for group in senario_dict['agent_groups'].values():
            for agent_prop in group:
                agent = AgentProp(agent_prop)
                self.agent_prop_list.append(agent)


        if self.reset_mode =='random':
            self.ref_state_list = []
            for group in senario_dict['agent_groups'].values():
                for agent_prop in group:
                    agent = AgentProp(agent_prop)
                    state = AgentState()
                    state.x = agent.init_x
                    state.y = agent.init_y
                    state.theta = agent.init_theta
                    state.vel = agent.init_vel
                    self.ref_state_list.append(state)
        elif self.reset_mode =='init':
            self.ref_state_list = []
            for group in senario_dict['agent_groups'].values():
                for agent_prop in group:
                    agent = AgentProp(agent_prop)
                    state = AgentState()
                    state.x = agent.init_x
                    state.y = agent.init_y
                    state.theta = agent.init_theta
                    state.vel = agent.init_vel
                    self.ref_state_list.append(state)
        
        self.fence_list = []
        if 'fence_groups' in senario_dict.keys():
            for group in senario_dict['fence_groups'].values():
                for fence_prop in group:
                    fence = Fence(fence_prop)
                    self.fence_list.append(fence)

        # assign color for each agent
        self.agent_color_list = None
        self.cam_bound = None
        self.viewer = None
        self.fly_size = None
    
    def reset(self):
        # reset initial state list in random way or in inital way
        if self.reset_mode == 'random':
            for idx in range(self.fence_number):
                self.fence_list[idx].anchor[0] = random.uniform(self.field_range[0],self.field_range[1])
                self.fence_list[idx].anchor[1] = random.uniform(self.field_range[2],self.field_range[3])
            self.last_state_list = random_reset(field_range=self.field_range,
                                                state_before_reset=self.ref_state_list,
                                                fence_list = self.fence_list)
        elif self.reset_mode == 'init':
            self.last_state_list = copy.deepcopy(self.ref_state_list)
        else:
            print('reset_mode must be random or init but %s found'%self.reset_mode)
        
        self.total_time = 0.0
        self.last_obs_list = self.get_obs_list()
        self.agent_geom_list = None
        self.fence_geom_list = None
        return self.last_obs_list

    def get_state(self):
        return self.last_state_list
    
    def step(self, action):
        reward_list = self._step(action)

        done = False
        crashed_list = [ state.crashed for state in self.last_state_list ]
        info = {'reward_list':reward_list, 'time_stop':self.total_time > self.time_limit,'crashed':crashed_list}
        return self.last_obs_list,sum(reward_list),done,info
    
    def get_obs_list(self):
        obs_list = []
        for idx in range(self.agent_number):
            state = self.last_state_list[idx]
            ct = numpy.cos(state.theta)
            st = numpy.sin(state.theta)
            vx = ct*state.vel
            vy = st*state.vel
            
            min_dist = float('inf')
            for idx in range(self.fence_number):
                fence = self.fence_list[idx]
                fence_x = fence.anchor[0]
                fence_y = fence.anchor[1]
                dist = ((fence_x-state.x)**2 + (fence_y-state.y)**2)**0.5
                if dist<min_dist:
                    min_dist = dist
                    min_idx = idx
            the_nearest_fence = self.fence_list[min_idx]
            the_nearest_radius = the_nearest_fence.radius
            abs_angle = numpy.arctan2(the_nearest_fence.y-state.y, the_nearest_fence.x-state.x)
            ralated_angle = (abs_angle-state.theta)%(2*3.1415)

            pos = numpy.array([state.x, state.y, ct, st, vx, vy, min_dist, the_nearest_radius, ralated_angle])    

            obs_list.append(pos)

        for idx in range(self.enermy_number):
            state = self.last_state_list[idx+self.agent_number]
            ct = numpy.cos(state.theta)
            st = numpy.sin(state.theta)
            vx = ct*state.vel
            vy = st*state.vel
            pos = numpy.array([state.x, state.y, ct, st, vx, vy])
            obs_list.append(pos)
        
        for idx in range(self.fence_number):
            fence = self.fence_list[idx]
            fence_x = fence.anchor[0]
            fence_y = fence.anchor[1]
            radius = fence.radius
            pos = numpy.array([fence_x,fence_y,radius])
            obs_list.append(pos)


        return obs_list

    def render(self, mode='human'):
        if self.agent_color_list is None:
            self.agent_color_list = []
            for idx in range(self.agent_number + self.enermy_number):
                if self.agent_prop_list[idx].R_B == 'red':
                    self.agent_color_list.append((1,0,0))
                else:
                    self.agent_color_list.append((0,0,1))
                
        if self.cam_bound is None:
            center_x = (self.field_range[0]+self.field_range[1])/2.0
            center_y = (self.field_range[2]+self.field_range[3])/2.0
            length_x = (self.field_range[1]-self.field_range[0])
            length_y = (self.field_range[3]-self.field_range[2])
            # 1.4 times of field range for camera range
            self.cam_bound = [center_x - 0.7*length_x, 1.4*length_x, center_y - 0.7*length_y, 1.4*length_y ]
        if self.viewer is None:
            from . import rendering 

            import pyglet
            screen = pyglet.canvas.get_display().get_default_screen()
            max_width = int(screen.width * 0.9) 
            max_height = int(screen.height * 0.9)
            if self.cam_bound[1]/self.cam_bound[3]>max_width/max_height:
                screen_width = max_width
                screen_height  = max_width/(self.cam_bound[1]/self.cam_bound[3])
            else:
                screen_height = max_height
                screen_width  = max_height*(self.cam_bound[1]/self.cam_bound[3])
            self.viewer = rendering.Viewer(int(screen_width),int(screen_height))
            self.viewer.set_bounds(self.cam_bound[0],self.cam_bound[0]+self.cam_bound[1],self.cam_bound[2],self.cam_bound[2]+self.cam_bound[3])
            self.fly_size = min(self.cam_bound[1],self.cam_bound[3])/60
        # create rendering geometry
        if self.fence_geom_list is None:
            # import rendering only if we need it (and don't import for headless machines)
            from . import rendering
            self.fence_geom_list = []
            
            for idx in range(self.fence_number):
                fence_geom = {}
                total_xform = rendering.Transform()
                fence_geom['total_xform'] = total_xform
                fence = self.fence_list[idx]
                
                geom = rendering.make_circle(radius=fence.radius, filled=fence.filled)
                
                geom.set_color(0,0,0,alpha = 1)
                xform = rendering.Transform()
                geom.add_attr(xform)
                geom.add_attr(total_xform)
                fence_geom['fence']=(geom,xform)


                self.fence_geom_list.append(fence_geom)

            self.viewer.geoms = []
            for fence_geom in self.fence_geom_list:
                self.viewer.add_geom(fence_geom['fence'][0])
        if self.agent_geom_list is None:
            # import rendering only if we need it (and don't import for headless machines)
            from . import rendering
            self.agent_geom_list = []
            
            for idx in range(self.agent_number + self.enermy_number):
                agent_geom = {}
                total_xform = rendering.Transform()
                agent_geom['total_xform'] = total_xform

                agent_color = self.agent_color_list[idx]
               
                
                half_l = self.fly_size/2.0
                half_w = self.fly_size/3.0/2.0
                geom = rendering.make_polygon([[half_l,0],[-half_l,half_w],[-half_l,-half_w]])
                geom.set_color(*agent_color,alpha = 10)
                xform = rendering.Transform()
                geom.add_attr(xform)
                geom.add_attr(total_xform)
                agent_geom['car']=(geom,xform)


                self.agent_geom_list.append(agent_geom)

            
            for agent_geom in self.agent_geom_list:
                self.viewer.add_geom(agent_geom['car'][0])

        for agent_idx,agent_geom in enumerate(self.agent_geom_list):
            agent_state = self.last_state_list[agent_idx]

            agent_geom['total_xform'].set_rotation(agent_state.theta)
            agent_geom['total_xform'].set_translation(agent_state.x*1.0,agent_state.y*1.0)

        for fence_idx,fence_geom in enumerate(self.fence_geom_list):
            fence = self.fence_list[fence_idx]
            fence_geom['total_xform'].set_translation(fence.anchor[0],fence.anchor[1])

        
        
        time_str = "{0:.2f}s".format(self.total_time)
        return self.viewer.render(time=time_str,return_rgb_array = mode=='rgb_array')


    def _step(self, action):
        step_time = 0.0
        old_state = copy.deepcopy(self.last_state_list)
        self._apply_action(action)
        for _ in range(self.sim_step):
            step_time += self.sim_t
            self.total_time += self.sim_t
            self._integrate_state(step_time)
            if self.viewer is not None:
                self.render()
        
        reward_list = self._calc_reward_list(old_state, self.last_state_list)

        self.last_obs_list = self.get_obs_list()
 
        return reward_list
    
    def _apply_action(self, action):
        action = numpy.clip(action, -1.0,1.0)
        # set applied forces

        for idx in range(self.agent_number + self.enermy_number):
            state_a = self.last_state_list[idx]
            state_a.last_x = state_a.x
            state_a.last_y = state_a.y
            state_a.last_theta = state_a.theta
            state_a.last_vel = state_a.vel
            state_a.next_phi = self.agent_prop_list[idx].K_phi * action[idx]
            state_a.crashed = False
    
    def _integrate_state(self, step_time):
        for idx in range(self.agent_number + self.enermy_number):
            state = self.last_state_list[idx]
            prop = self.agent_prop_list[idx]
            #state.theta = (state.last_theta + state.next_phi)%(2*3.14159)
            delay = (state.next_phi - numpy.sin(state.next_phi))*prop.Turn_radius
            forward = prop.True_velocity*self.step_t
            switch_time = abs(2*state.next_phi*prop.Turn_radius/prop.True_velocity)
            temp_angle = prop.True_velocity*step_time/prop.Turn_radius
            sign = -1 if state.next_phi<0 else 1
                
            s_move = prop.Turn_radius*(temp_angle-2*state.next_phi*sign)
            if step_time<switch_time:
                move_x = prop.Turn_radius*(numpy.sin(temp_angle))
                move_y = prop.Turn_radius*(1-numpy.cos(temp_angle))*sign
                if step_time<switch_time*0.9:
                    move_theta = temp_angle*sign
                else:
                    c = (step_time/switch_time-0.9)*10
                    move_theta = temp_angle*sign*(1-c) + state.next_phi * c
            else:
                
                move_x = prop.Turn_radius*numpy.sin(2*state.next_phi)*sign + s_move * numpy.cos(state.next_phi)
                move_y = prop.Turn_radius*(1 - numpy.cos(2*state.next_phi))*sign + s_move * numpy.sin(state.next_phi)
                move_theta = state.next_phi
            
            state.x = state.last_x + move_x * numpy.cos(state.last_theta) - move_y * numpy.sin(state.last_theta)
            state.y = state.last_y + move_x * numpy.sin(state.last_theta) + move_y * numpy.cos(state.last_theta)
            state.theta = (state.last_theta + move_theta)%(2*3.14159)

            for fence in self.fence_list:
                dist = ((fence.anchor[0]-state.x)**2 + (fence.anchor[1]-state.y)**2)**0.5
                if dist<fence.radius:
                    state.crashed = True
        ##TODO
    
    def _calc_reward_list(self, old_state, new_state):
        ##TODO
        ##self.reward_coef['XXXX']*XXXX
        return [0 for idx in range(self.agent_number+self.enermy_number)]