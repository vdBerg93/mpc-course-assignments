import numpy as np
from sim.sim1d import sim_run

# Simulator options.
options = {}
options['FIG_SIZE'] = [8,8]
options['FULL_RECALCULATE'] = False

class ModelPredictiveControl:
    def __init__(self):
        self.horizon = 20
        self.dt = 0.2
        self.vmax = 10/3.6
        self.w_dist = 1
        self.w_speed = 10000
        # Reference or set point the controller will achieve.
        self.reference = [50, 0, 0]

    def plant_model(self, prev_state, dt, pedal, steering):
        x_t = prev_state[0]
        v_t = prev_state[3] # m/s
        x_t_1 = x_t + v_t*dt
        v_t_1 = 0.95*v_t + pedal
        return [x_t_1, 0, 0, v_t_1]

    def cost_function(self,u, *args):
        state = args[0]
        ref = args[1]
        cost = 0
        for u_i in u:
            state = self.plant_model(state, self.dt, u_i, None)
            d_goal = ref[0]-state[0]
            err_speed = state[3]-self.vmax
            cost += pow(d_goal,2) + (err_speed>0)*self.w_speed*err_speed
        return cost

sim_run(options, ModelPredictiveControl)
