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
            self.state_t = tf.placeholder(dtype=tf.float32, shape=[2])
            self.x_t = [tf.gather(self.state_t, 0)] # x
            self.y_t = [tf.gather(self.state_t, 1)] # y
            self.v_t = tf.Variable(np.random.normal(0.0, scale=1, size=N), dtype=tf.float32)
            self.s_t = tf.Variable(np.random.normal(0.0, scale=1, size=N), dtype=tf.float32)

            self.loss_t = 0.0
            for i in range(1, N):
                p = i - 1
                self.x_t.append(self.x_t[p] + self.v_t[p] * tf.cos(self.s_t[p]) * dt)
                self.y_t.append(self.y_t[p] + self.v_t[p] * tf.sin(self.s_t[p]) * dt)
                self.loss_t += (
                    tf.square(self.x_t[p] - 0.5) +
                    tf.square(self.y_t[p] - 0.5)
                )
            for i in range(0, N-2):
                self.loss_t += (
                    tf.square(tf.gather(self.v_t, i+1) - tf.gather(self.v_t, i)) +
                    tf.square(tf.gather(self.s_t, i+1) - tf.gather(self.s_t, i))
                )

            v_b = 0.8
            b = [np.ones(N) * -v_b, np.ones(N) * v_b]
            #b[0][0] = 1
            print(self.state_t[0])


            self.opt = tf.contrib.opt.ScipyOptimizerInterface(self.loss_t,
                var_to_bounds={
                    self.v_t: b,
                })
            self.init_op = tf.global_variables_initializer()

    def initialize(self):
        if self.graph is None:
            self.build_graph()
            self.sess = tf.Session(graph=self.graph)
            self.sess.run(self.init_op)

    def solve(self, state):
        self.opt.minimize(self.sess, feed_dict={self.state_t: state})
        ops_list = [self.x_t, self.y_t]
        next_states = self.sess.run(ops_list, feed_dict={self.state_t: state})
        x, y = next_states
        next_state = np.array([x[1], y[1]], dtype=np.float32)
        opt_actuators = self.sess.run([self.v_t, self.s_t], feed_dict={self.state_t: state})
        return [next_state, opt_actuators]

dt = 0.01
mpc = MPC(N=25, dt=dt)
mpc.initialize()

print('Begin')
then = time()

x_vals = []
y_vals = []
s_vals = []
v_vals = []
state = np.array([0, 0], dtype=np.float32)

steps = 150
for iteration in range(steps):
    state, opt_actuators = mpc.solve(state)
    x_vals.append(state[0])
    y_vals.append(state[1])
    v_vals.append(opt_actuators[0][0])
    s_vals.append(opt_actuators[1][0])

now = time()
print('Complete! %fms' % (((now-then)/steps)*1000.))

fig, ax = plt.subplots(3, 1)
fig.set_size_inches(12, 12)
ax = ax.flatten()
t = np.arange(0, len(x_vals)) * dt

ax[0].plot(x_vals, y_vals)
ax[0].title.set_text("Position")
ax[0].grid()

ax[1].plot(t, s_vals)
ax[1].title.set_text("Steer angle")
ax[1].grid()

ax[2].plot(t, v_vals)
ax[2].title.set_text("Drive velocity")
ax[2].grid()

fig.tight_layout()
plt.show()
