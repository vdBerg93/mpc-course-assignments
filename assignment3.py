import numpy as np
from sim.sim2d import sim_run

# Simulator options.
options = {}
options['FIG_SIZE'] = [8,8]
options['OBSTACLES'] = True

class ModelPredictiveControl:
    def __init__(self):
        self.horizon = 10
        self.dt = 0.2

        # Reference or set point the controller will achieve.
        self.reference1 = [10, 0, 0]
        self.reference2 = None

        self.x_obs = 5
        self.y_obs = 0.1
        # cost weights
        self.w_pos = 10
        self.w_head = 1
        self.w_accel = 0.1

    def plant_model(self,prev_state, dt, pedal, steering):
        x_t = prev_state[0] + prev_state[3]*np.cos(prev_state[2])*dt
        y_t = prev_state[1] + prev_state[3]*np.sin(prev_state[2])*dt
        psi_t = prev_state[2] + prev_state[3]*(np.tan(steering)/2.5)*dt
        v_t = prev_state[3] + pedal*dt - prev_state[3]/25

        return [x_t, y_t, psi_t, v_t]

    def cost_function(self,u, *args):
        state = args[0]
        ref = args[1]
        cost = 0.0
        for k in range(0,self.horizon):
            state = self.plant_model(state, self.dt, u[k*2], u[k*2+1])
            d_goal = np.power(state[0]-ref[0],2) + np.power(state[1]-ref[1],2)
            d_obs = np.power(state[0]-self.x_obs,2) + np.power(state[1]-self.y_obs,2)

            #cost += (d_obs<2)*d_obs
            cost += 200/d_obs
            cost += self.w_pos*(d_goal)       # distance
            cost += abs(state[2]-ref[2])**2     # heading
            #cost += abs(u[k*2])**2              # acceleration
            #cost += (state[3]<0)*state[3]**2    #reverse
        return cost

sim_run(options, ModelPredictiveControl)
