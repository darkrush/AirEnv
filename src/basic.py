# properties of agent entities
class AgentProp(object):
    def __init__(self,agent_prop = None):
        self.R_cover     = 1e4  # minimal distance cover the enermy
        self.Velocity   = 51.25   # velocity of the plane
        self.True_velocity = 3033.97/40 #3033.97m/40s
        self.Turn_radius = 817.86
        self.K_phi      = 115/180*3.1415   # coefficient of the angle which the fly-point can turn
        self.R_B = 'red'

        self.init_x = 0
        self.init_y = 0
        self.init_vel = 0
        self.init_theta = 0

        if agent_prop is not None:
            for k,v in agent_prop.items():
                self.__dict__[k] = v

class AgentState(object):
    def __init__(self):
        #center point position in x,y axis
        self.x = 0
        self.y = 0
        #linear velocity of back point
        self.vel = 0
        # direction of car axis
        self.theta = 0

        self.crashed = False

        self.last_x = 0
        self.last_y = 0
        self.last_theta = 0
        self.last_vel = 0

        self.next_phi = 0


    
    def __str__(self):
        return 'x : '+str(self.x)+' y : '+str(self.y)+' theta : '+str(self.theta)
    def __repr__(self):
        return self.__str__()


class Fence(object):
    def __init__(self,fence_prop):# anchor=[0,0], rotation = 0, vertices=([0,0],), close = False, filled = False, color = [0.0, 0.0, 0.0]):
        
        # the anchor point in global coordinate
        self.anchor = [fence_prop['anchor_x'],fence_prop['anchor_y']]

        self.type = fence_prop['type']

        if self.type == 'circle':
            self.radius = fence_prop['radius']
        else:
            # the rotation angle by radian global coordinate
            self.rotation = fence_prop['rotation']
            # the coordinate of vertices related to anchor
            self.vertices = zip(fence_prop['vertices_x'],fence_prop['vertices_y'])
            # A close fence means a fence between vertices[-1] and vertices[0], forced to be True if filled 
            self.close = fence_prop['close'] or fence_prop['filled']
            self.calc_vertices()
        
        # Fill the fence by color inside if True
        self.filled = fence_prop['filled']
        

        

    def calc_vertices(self):
        self.global_vertices = []
        for v in self.vertices:
            c = np.cos(self.rotation)
            s = np.sin(self.rotation)
            g_v_x = v[0]*c - v[1]*s +self.anchor[0]
            g_v_y = v[1]*c + v[0]*s +self.anchor[1]
            self.global_vertices.append(np.array([g_v_x,g_v_y]))
        if self.close:
            self.global_vertices.append(self.global_vertices[0])