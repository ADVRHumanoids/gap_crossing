#!/usr/bin/env python
from casadi import *
import matlogger2.matlogger as matl

logger = matl.MatLogger2('/tmp/RAL_190957_replies_log')
logger.setBufferMode(matl.BufferMode.CircularBuffer)

ns = 5  # number of steps

nc = 2  # number of contacts

n_com = 2
n_p = 2*nc
n_n = 2*nc
n_f = 2*nc

# Bounds and initial guess
com_min = np.array([-10., 0.])
com_max = np.array([10., 2.])
p_min = np.array([-10., -10., -10., -10.])
p_max = np.array([10., 10., 10., 10.])
n_min = np.array([-1., -1., -1., -1.])
n_max = np.array([1., 1., 1., 1.])
f_min = np.array([-1000., -1000., -1000., -1000.])
f_max = np.array([1000., 1000., 1000., 1000.])

com_init = np.array([0.5, 1.])
p_init = np.array([0., 0., 1., 0.])
n_init = np.array([0., 1., 0., 1.])
f_init = np.array([0., 250., 0., 250.])

com_final = np.array([7.5, 1.])

# Symbolic variables
com = SX.sym('com', n_com)
p = SX.sym('p', n_p)
n = SX.sym('n', n_n)
f = SX.sym('f', n_f)

point = SX.sym('point', 2)
normal = SX.sym('normal', 2)
force = SX.sym('force', 2)

#  Centroidal Statics
tau_CS = SX.sym('tau_CS', 3)
delta = p - vertcat(com, com)
g = -9.81
m = 50.

tau_CS[0:2] = f[0:2] + f[2:4]
tau_CS[1] += m*g
tau_CS[2] = delta[0]*f[1] - delta[1]*f[0] + delta[2]*f[3] - delta[3]*f[2]

F_CS = Function('F_CS', [p, com, f], [tau_CS], ['p', 'com', 'f'], ['tau_CS'])

# Environment
Sc = SX.sym('Sc', 1)
# Sc = point[1]  # flat ground
Sc = point[1] + atan(1e6*(point[0]-3.0)) - atan(1e6*(point[0]-4.0))  # flat ground with gap

F_env = Function('F_env', [point], [Sc], ['point'], ['Sc'])

n_env = jacobian(Sc, point)/norm_1(jacobian(Sc, point))
N_env = Function('N_env', [point], [n_env], ['point'], ['n_env'])

# Friction cones
friction_cone = SX.sym('friction_cone', 3)

F_thr = 0.0
mu = 0.5

friction_cone[0] = F_thr - dot(force, normal)

# force_t = force - dot(force, normal)*normal
# friction_cone[1] = pow(force_t[0], 2) + pow(force_t[1], 2) - pow(mu*dot(force, normal), 2)

friction_cone[1] = -normal[1]*force[0] + normal[0]*force[1] - mu*dot(force, normal)
friction_cone[2] = normal[1]*force[0] - normal[0]*force[1] - mu*dot(force, normal)

F_fr = Function('F_fr', [force, normal], [friction_cone], ['force', 'normal'], ['friction_cone'])

# Start with an empty NLP
NV = (n_com + n_p + n_n + n_f) * ns
V = MX.sym('V', NV)

# NLP vars bounds and init guess
v_min = []
v_max = []
v_init = []
g_min = []
g_max = []

# offset in v
offset = 0

# "Lift" initial conditions
COM = []
P = []
N = []
F = []

# Formulate the NLP
for k in range(ns):

    # COM at k-th node
    COM.append(V[offset:offset+n_com])

    if k == 0:
        v_min += com_min.tolist()
        v_max += com_init.tolist()
    elif k == (ns - 1):
        v_min += com_final.tolist()
        v_max += com_final.tolist()
    else:
        v_min += com_min.tolist()
        v_max += com_max.tolist()

    v_init += com_init.tolist()

    offset += n_com

    # Contacts at k-th node
    P.append(V[offset:offset + n_p])

    v_min += p_min.tolist()
    v_max += p_max.tolist()

    v_init += p_init.tolist()

    offset += n_p

    # Normals at k-th node
    N.append(V[offset:offset + n_n])

    v_min += n_min.tolist()
    v_max += n_max.tolist()

    v_init += n_init.tolist()

    offset += n_n

    # Forces at k-th node
    F.append(V[offset:offset + n_f])

    v_min += f_min.tolist()
    v_max += f_max.tolist()

    v_init += f_init.tolist()

    offset += n_f

assert offset == NV

# Create NLP
J = MX([0])
g = []

com_history = MX(Sparsity.dense(n_com, ns))
p_history = MX(Sparsity.dense(n_p, ns))
n_history = MX(Sparsity.dense(n_n, ns))
f_history = MX(Sparsity.dense(n_f, ns))

COM_final = MX([7., 1.])

stance_idx = 0

for k in range(ns):

    if stance_idx == 0:
        stance_idx = 2
    else:
        stance_idx = 0

    P1_k = P[k][0:2]
    P2_k = P[k][2:4]
    N1_k = N[k][0:2]
    N2_k = N[k][2:4]
    F1_k = F[k][0:2]
    F2_k = F[k][2:4]

    J += 1000.*dot(COM[k] - COM_final, COM[k] - COM_final)
    J += 10.*dot(F[k], F[k])

    Sc1_k = F_env(point=P1_k)['Sc']
    Sc2_k = F_env(point=P2_k)['Sc']

    n1_k = N_env(point=P1_k)['n_env']
    n2_k = N_env(point=P2_k)['n_env']

    tau_CS_k = F_CS(p=P[k], com=COM[k], f=F[k])['tau_CS']

    fr1_k = F_fr(force=F1_k, normal=N1_k)['friction_cone']
    fr2_k = F_fr(force=F2_k, normal=N2_k)['friction_cone']

    g += [tau_CS_k]
    g_min += np.zeros((3, 1)).tolist()
    g_max += np.zeros((3, 1)).tolist()

    g += [Sc1_k, Sc2_k]
    g_min += np.zeros((2, 1)).tolist()
    g_max += np.zeros((2, 1)).tolist()

    g += [COM[k]-P1_k, COM[k]-P2_k]
    g_min += np.array([-1., 0.5, -1., 0.5]).tolist()
    g_max += np.array([1., 1.5, 1., 1.5]).tolist()

    # g += [norm_2(COM[k]-P1_k), norm_2(COM[k]-P2_k)]
    # g_min += np.array([1.1, 1.1]).tolist()
    # g_max += np.array([1.5, 1.5]).tolist()

    # g += [COM[k][1]]
    # g_min += np.array([1.]).tolist()
    # g_max += np.array([1.]).tolist()

    if k < ns-1:
        g += [P[k][stance_idx:(stance_idx+2)]-P[k+1][stance_idx:(stance_idx+2)]]
        g_min += np.zeros((2, 1)).tolist()
        g_max += np.zeros((2, 1)).tolist()

    g += [N1_k[0]-n1_k[0], N1_k[1]-n1_k[1], N2_k[0]-n2_k[0], N2_k[1]-n2_k[1]]
    g_min += np.zeros((4, 1)).tolist()
    g_max += np.zeros((4, 1)).tolist()

    g += [fr1_k, fr2_k]
    g_min += np.array([-1000., -1000., -1000., -1000., -1000., -1000.]).tolist()
    g_max += np.zeros((6, 1)).tolist()

    g += [fr1_k[0], fr2_k[0]]
    g_min += np.array([-1000., -1000.]).tolist()
    g_max += np.zeros((2, 1)).tolist()

    com_history[0:n_com, k] = COM[k]
    p_history[0:n_p, k] = P[k]
    f_history[0:n_f, k] = F[k]
    n_history[0:n_n, k] = N[k]

g = vertcat(*g)
v_init = vertcat(*v_init)
g_min = vertcat(*g_min)
g_max = vertcat(*g_max)
v_min = vertcat(*v_min)
v_max = vertcat(*v_max)

# Create an NLP solver
prob = {'f': J, 'x': V, 'g': g}
opts = {'ipopt.tol': 1e-3,
        'ipopt.max_iter': 100000,
        'ipopt.linear_solver': 'ma57'}
solver = nlpsol('solver', 'ipopt', prob, opts)

# Solve the NLP
sol = solver(x0=v_init, lbx=v_min, ubx=v_max, lbg=g_min, ubg=g_max)
w_opt = sol['x'].full().flatten()

# Plot the solution
com_hist = Function("com_hist", [V], [com_history])
com_hist_value = com_hist(w_opt).full()
p_hist = Function("q_hist", [V], [p_history])
p_hist_value = p_hist(w_opt).full()
n_hist = Function("n_hist", [V], [n_history])
n_hist_value = n_hist(w_opt).full()
f_hist = Function("f_hist", [V], [f_history])
f_hist_value = f_hist(w_opt).full()


logger.add('com', com_hist_value)
logger.add('p', p_hist_value)
logger.add('n', n_hist_value)
logger.add('F', f_hist_value)
logger.add('ns', ns)

del logger
