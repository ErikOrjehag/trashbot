import numpy as np
import scipy as sp
import tensorflow as tf
import matplotlib.pyplot as plt
from time import time

class MPC:
    def __init__(self, N=25, dt=0.01):
        self.N = N
        self.dt = dt
        self.graph = None

    def build_graph(self):
        N = self.N
        dt = self.dt
        self.graph = tf.Graph()
        with self.graph.as_default():
            self.state_t = tf.placeholder(dtype=tf.float32, shape=[4])
            self.x_t = [tf.gather(self.state_t, 0)] # x
            self.y_t = [tf.gather(self.state_t, 1)] # y
            self.s_t = [tf.gather(self.state_t, 2)] # steer
            self.v_t = [tf.gather(self.state_t, 3)] # velocity
            self.a_t = tf.Variable(np.random.normal(0.0, scale=1, size=N), dtype=tf.float32)
            self.w_t = tf.Variable(np.random.normal(0.0, scale=1, size=N), dtype=tf.float32)

            self.loss_t = 0.0
            for i in range(1, N):
                p = i - 1
                self.x_t.append(self.x_t[p] + self.v_t[p] * tf.cos(self.s_t[p]) * dt)
                self.y_t.append(self.y_t[p] + self.v_t[p] * tf.sin(self.s_t[p]) * dt)
                self.s_t.append(self.s_t[p] + self.w_t[p] * dt)
                self.v_t.append(self.v_t[p] + self.a_t[p] * dt)
                self.loss_t += (
                    tf.square(self.x_t[p] - 0.5) +
                    tf.square(self.y_t[p] - 0.5)
                )

            for i in range(0, N-2):
                self.loss_t += (
                    tf.square(tf.gather(self.a_t, i+1) - tf.gather(self.a_t, i)) +
                    tf.square(tf.gather(self.w_t, i+1) - tf.gather(self.w_t, i))
                )

            a_b = 8.0
            w_b = 12.0
            v_b = 0.8

            inequalities = []
            for i in range(len(self.v_t)):
                inequalities.append(v_b - self.v_t[i])
                inequalities.append(v_b + self.v_t[i])

            self.opt = tf.contrib.opt.ScipyOptimizerInterface(self.loss_t,
                method='SLSQP',
                #options={'maxiter': 100},
                inequalities=inequalities,
                var_to_bounds={
                    self.a_t: [np.ones(N) * -a_b, np.ones(N) * a_b],
                    self.w_t: [np.ones(N) * -w_b, np.ones(N) * w_b]
                })
            self.init_op = tf.global_variables_initializer()

    def initialize(self):
        if self.graph is None:
            self.build_graph()
            #, config=tf.ConfigProto(log_device_placement=True)
            self.sess = tf.Session(graph=self.graph)
            self.sess.run(self.init_op)

    def solve(self, state):
        self.opt.minimize(self.sess, feed_dict={self.state_t: state})
        ops_list = [self.x_t, self.y_t, self.s_t, self.v_t]
        next_states = self.sess.run(ops_list, feed_dict={self.state_t: state})
        x, y, s, v = next_states
        next_state = np.array([x[1], y[1], s[1], v[1]], dtype=np.float32)
        opt_actuators = self.sess.run([self.a_t, self.w_t], feed_dict={self.state_t: state})
        return [next_state, opt_actuators]

dt = 0.02
mpc = MPC(N=25, dt=dt)
mpc.initialize()

print('Begin')
then = time()

x_vals = [0]
y_vals = [0]
s_vals = [0]
v_vals = [0]
a_vals = []
w_vals = []
state = np.array([x_vals[0], y_vals[0], s_vals[0], v_vals[0]], dtype=np.float32)

steps = 100
for iteration in range(steps):
    state, opt_actuators = mpc.solve(state)
    x_vals.append(state[0])
    y_vals.append(state[1])
    s_vals.append(state[2])
    v_vals.append(state[3])
    a_vals.append(opt_actuators[0][0])
    w_vals.append(opt_actuators[1][0])

a_vals.append(0)
w_vals.append(0)

now = time()
print('Complete! %fms' % (((now-then)/steps)*1000.))

fig, ax = plt.subplots(3, 2)
fig.set_size_inches(12, 12)
ax = ax.flatten()
t = np.arange(0, len(x_vals)) * dt

ax[0].plot(x_vals, y_vals)
ax[0].title.set_text("Position")
ax[0].grid()

ax[1].plot(t, s_vals)
ax[1].title.set_text("Steer angle")
ax[1].grid()

ax[2].plot(t, a_vals)
ax[2].title.set_text("Drive acceleration")
ax[2].grid()

ax[3].plot(t, w_vals)
ax[3].title.set_text("Steer velocity")
ax[3].grid()

ax[4].plot(t, v_vals)
ax[4].title.set_text("Drive velocity")
ax[4].grid()

fig.tight_layout()
plt.show()
