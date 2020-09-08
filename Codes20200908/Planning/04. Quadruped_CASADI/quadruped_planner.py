#!/usr/bin/env python
from casadi import *

from mpl_toolkits.mplot3d import Axes3D
import numpy
import matplotlib.pyplot as plt
import matplotlib.axes
import time

ns = 40 # number of steps

nc = 4  # number of contacts

dim_comF = 3
dim_comB = 3
dim_p = 3*nc
dim_n = 3*nc
dim_f = 3*nc

mB = 50.
mF = 50.

F = (mB+mF) * 9.81
f = F/4

# Bounds and initial guess
com_min = np.array([-10., -10., 0.3])
com_max = np.array([10., 10., 2.])
p_min = np.array([-10., -10., 0., -10., -10., 0., -10., -10., 0., -10., -10., 0.])
p_max = np.array([10., 10., 0., 10., 10., 0., 10., 10., 0., 10., 10., 0.])
n_min = np.array([-1., -1., -1., -1., -1., -1., -1., -1., -1., -1., -1., -1.])
n_max = np.array([1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1.])
f_min = np.array([-1000., -1000., -1000., -1000., -1000., -1000., -1000., -1000., -1000., -1000., -1000., -1000.])
f_max = np.array([1000., 1000., 1000., 1000., 1000., 1000., 1000., 1000., 1000., 1000., 1000., 1000.])

comF_init = np.array([2., 0.5, 0.5])
comB_init = np.array([0., 0.5, 0.5])
p_init = np.array([0., 0., 0., 0., 1., 0., 2., 0., 0., 2., 1., 0.])
n_init = np.array([0., 0., 1., 0., 0., 1., 0., 0., 1., 0., 0.,  1.])
f_init = np.array([0., 0., f, 0., 0., f, 0., 0., f, 0., 0., f])

comF_final = np.array([5., 1.5, 0.5])
comB_final = np.array([3., 1.5, 0.5])

# Gap specifications
x_min = 2.25
x_max = 2.35
epsilon = 0.01
gap_min = x_min - epsilon
gap_max = x_max + epsilon


# Symbolic variables
comF = SX.sym('comF', dim_comF)
comB = SX.sym('comB', dim_comB)
p = SX.sym('p', dim_p)
n = SX.sym('n', dim_n)
f = SX.sym('f', dim_f)

point = SX.sym('point', 3)
normal = SX.sym('normal', 3)
force = SX.sym('force', 3)

# Centroidal Statics
g_vec = SX([0.,0.,-9.81,0.,0.,0.])
I3 = SX.eye(3)
tau_CS = SX.sym('tau_CS', 6)
mF = 50.
mB = 50.

com_whole = (comF*mF + comB * mB)/(mF + mB)
delta = p[0:12] - vertcat(com_whole, com_whole, com_whole, com_whole)

I3 = SX.eye(3)
I3cat = horzcat(I3, I3, I3, I3)

S1 = skew(delta[0:3])
S2 = skew(delta[3:6])
S3 = skew(delta[6:9])
S4 = skew(delta[9:12])

Scat = horzcat(S1, S2, S3, S4)

G_CD = vertcat(I3cat, Scat)

tau_CS = (mF + mB) * g_vec + mtimes(G_CD, f)

F_CS = Function('F_CS', [p, comF, comB, f], [tau_CS] , ['p', 'comF', 'comB', 'f'], ['tau_CS'])


# Environment
Sc = SX.sym('Sc', 1)
Sc = point[2]  # flat ground
#Sc = point[2] - atan(1e6*(point[0]-2.3)) + atan(1e6*(point[0]-2.2))  # flat ground with gap

F_env = Function('F_env', [point], [Sc], ['point'], ['Sc'])

n_env = jacobian(Sc, point)/norm_1(jacobian(Sc, point))
N_env = Function('N_env', [point], [n_env], ['point'], ['n_env'])

# Friction cones
friction_cone = SX.sym('friction_cone', 2)

F_thr = 0.0
mi = 0.5

force_n = dot(force, normal) * normal
force_t = force - force_n

friction_cone[0] = F_thr - dot(force, normal)
friction_cone[1] = norm_1(force_t) - mi * dot(force, normal)

F_fr = Function('F_fr', [force, normal], [friction_cone], ['force', 'normal'], ['friction_cone'])

# CoM to Feet distances
distF = SX.sym('distF', 2)
distB = SX.sym('distB', 2)

distF[0] = dot(comF - p[6:9], comF - p[6:9])
distF[1] = dot(comF - p[9:12], comF - p[9:12])

distB[0] = dot(comB - p[0:3], comB - p[0:3])
distB[1] = dot(comB - p[3:6], comB - p[3:6])

dist_coms_feet = SX.sym('dist_coms_feet', 4)
dist_coms_feet = vertcat(distB, distF)

F_sqdist = Function('F_sqdist', [p, comB, comF], [dist_coms_feet], ['p', 'comB', 'comF'], ['dist_coms_feet'])
# Start with an empty NLP
NV = (dim_comF + dim_comB + dim_p + dim_f) * ns
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
COMF = []
COMB = []
P = []
N = []
F = []

# Formulate the NLP
for k in range(ns):
    # COMB at k-th node
    COMB.append(V[offset:offset+dim_comB])

    if k == 0:
        v_min += comB_init.tolist()
        v_max += comB_init.tolist()
    elif k == (ns - 1):
        v_min += comB_final.tolist()
        v_max += comB_final.tolist()
    else:
        v_min += com_min.tolist()
        v_max += com_max.tolist()

    v_init += comB_init.tolist()

    offset += dim_comB


    # COMF at k-th node
    COMF.append(V[offset:offset+dim_comF])

    if k == 0:
        v_min += comF_init.tolist()
        v_max += comF_init.tolist()
    elif k == (ns - 1):
        v_min += comF_final.tolist()
        v_max += comF_final.tolist()
    else:
        v_min += com_min.tolist()
        v_max += com_max.tolist()

    v_init += comF_init.tolist()

    offset += dim_comF

    # Contacts at k-th node
    P.append(V[offset:offset + dim_p])

    if k == 0:
        v_min += p_init.tolist()
        v_max += p_init.tolist()
    else:
        v_min += p_min.tolist()
        v_max += p_max.tolist()

    v_init += p_init.tolist()

    offset += dim_p

    # Forces at k-th node
    F.append(V[offset:offset + dim_f])

    if k == 0:
        v_min += f_init.tolist()
        v_max += f_init.tolist()
    else:
        v_min += f_min.tolist()
        v_max += f_max.tolist()

    v_init += f_init.tolist()

    offset += dim_f

assert offset == NV

# Create NLP
# min_V J(V)
# s.t. g(V) <= 0
J = MX([0]) 
g = []

comF_history = MX(Sparsity.dense(dim_comF, ns))
comB_history = MX(Sparsity.dense(dim_comB, ns))
p_history = MX(Sparsity.dense(dim_p, ns))
f_history = MX(Sparsity.dense(dim_f, ns))

COMF_final = MX(comF_final.tolist())
COMB_final = MX(comB_final.tolist())


for k in range(ns):
    COMF_k = COMF[k]
    COMB_k = COMB[k]
    P_BR_k = P[k][0:3]
    P_BL_k = P[k][3:6]
    P_FR_k = P[k][6:9]
    P_FL_k = P[k][9:12]
    F_BR_k = F[k][0:3]
    F_BL_k = F[k][3:6]
    F_FR_k = F[k][6:9]
    F_FL_k = F[k][9:12]

    J += 1000.*dot(COMF_k - COMF_final, COMF_k - COMF_final)
    J += 1000.*dot(COMB_k - COMB_final, COMB_k - COMB_final)
    J += 10.*dot(F[k], F[k])

    Sc_BR_k = F_env(point=P_BR_k)['Sc']
    Sc_BL_k = F_env(point=P_BL_k)['Sc']
    Sc_FR_k = F_env(point=P_FR_k)['Sc']
    Sc_FL_k = F_env(point=P_FL_k)['Sc']

    N_BR_k = N_env(point=P_BR_k)['n_env']
    N_BL_k = N_env(point=P_BL_k)['n_env']
    N_FR_k = N_env(point=P_FR_k)['n_env']
    N_FL_k = N_env(point=P_FL_k)['n_env']

    tau_CS_k = F_CS(p=P[k], comF=COMF_k, comB=COMB_k, f=F[k])['tau_CS']

    fr_BR_k = F_fr(force=F_BR_k, normal=N_BR_k)['friction_cone']
    fr_BL_k = F_fr(force=F_BL_k, normal=N_BL_k)['friction_cone']
    fr_FR_k = F_fr(force=F_FR_k, normal=N_FR_k)['friction_cone']
    fr_FL_k = F_fr(force=F_FL_k, normal=N_FL_k)['friction_cone']
    
    # tau_CS_k = 0
    g += [tau_CS_k]
    g_min += np.zeros((6, 1)).tolist()
    g_max += np.zeros((6, 1)).tolist()
    
    # Sc = 0 for all feet
    g += [Sc_BR_k, Sc_BL_k, Sc_FR_k, Sc_FL_k]
    g_min += np.zeros((4, 1)).tolist()
    g_max += np.zeros((4, 1)).tolist()
    
    # friction cone
    g += [fr_BR_k, fr_BL_k, fr_FR_k, fr_FL_k]
    g_min += np.array([-1000., -1000., -1000., -1000., -1000., -1000., -1000., -1000.]).tolist()
    g_max += np.zeros((8, 1)).tolist()

    # next step is bounded: P[k+1] in a circle centered in P[k]
    if k < ns-1 :
        g += [dot(P_BR_k - P[k+1][0:3], P_BR_k - P[k+1][0:3]), dot(P_BL_k - P[k+1][3:6], P_BL_k - P[k+1][3:6]), dot(P_FR_k - P[k+1][6:9], P_FR_k - P[k+1][6:9]), dot(P_FL_k - P[k+1][9:12], P_FL_k - P[k+1][9:12])]
        g_min += np.zeros((4, 1)).tolist()
        g_max += np.array([0.25, 0.25, 0.25, 0.25]).tolist()
       
    # feet distance is bounded
    
    g += [dot(P_BR_k - P_FR_k, P_BR_k - P_FR_k), dot(P_BL_k - P_FL_k, P_BL_k - P_FL_k), dot(P_BR_k - P_BL_k, P_BR_k - P_BL_k), dot(P_FR_k - P_FL_k, P_FR_k - P_FL_k), dot(P_BR_k - P_FL_k, P_BR_k - P_FL_k), dot(P_BL_k-P_FR_k, P_BL_k-P_FR_k)]
    g_min += np.array([1., 1., 0.25, 0.25, 1.25, 1.25]).tolist()
    g_max += np.array([9., 9., 2.25, 2.25, 11.25, 11.25]).tolist()
    '''
    g += [dot(P_BR_k - P_FR_k, P_BR_k - P_FR_k), dot(P_BL_k - P_FL_k, P_BL_k - P_FL_k), dot(P_BR_k - P_BL_k, P_BR_k - P_BL_k), dot(P_FR_k - P_FL_k, P_FR_k - P_FL_k)]
    g_min += np.array([1., 1., 0.25, 0.25]).tolist()
    g_max += np.array([9., 9., 2.25, 2.25]).tolist()
    '''
    # one and only one foot at time can be moved
    if k < ns-1 :
        move_BR = dot(P_BR_k - P[k+1][0:3], P_BR_k - P[k+1][0:3])
        move_BL = dot(P_BL_k - P[k+1][3:6], P_BL_k - P[k+1][3:6])
        move_FR = dot(P_FR_k - P[k+1][6:9], P_FR_k - P[k+1][6:9])
        move_FL = dot(P_FL_k - P[k+1][9:12], P_FL_k - P[k+1][9:12])

        g += [move_BR*move_BL, move_BR*move_FR, move_BR*move_FL, move_BL*move_FR, move_BL*move_FL, move_FR*move_FL]
        g_min += np.zeros((6, 1)).tolist()
        g_max += np.zeros((6, 1)).tolist()
        
    # COMs to feet constraints
    dist_coms_feet_k = F_sqdist(p=P[k], comB=COMB_k, comF=COMF_k)['dist_coms_feet']
    g += [dist_coms_feet_k]
    min_dist = 0.5
    max_dist = 0.5
    g_min += np.array([min_dist, min_dist, min_dist, min_dist]).tolist()
    g_max += np.array([max_dist, max_dist, max_dist, max_dist]).tolist()
    
    # Gap constraint
    g += [(P_BR_k[0]-gap_min)*(P_BR_k[0]-gap_max), (P_BL_k[0]-gap_min)*(P_BL_k[0]-gap_max), (P_FR_k[0]-gap_min)*(P_FR_k[0]-gap_max), (P_FL_k[0]-gap_min)*(P_FL_k[0]-gap_max)]
    g_min += np.zeros((4, 1)).tolist()
    g_max += np.array([10000, 10000, 10000, 10000]).tolist()
    
    comF_history[0:dim_comF, k] = COMF[k]
    comB_history[0:dim_comB, k] = COMB[k]
    p_history[0:dim_p, k] = P[k]
    f_history[0:dim_f, k] = F[k]

g = vertcat(*g)
v_init = vertcat(*v_init)
g_min = vertcat(*g_min)
g_max = vertcat(*g_max)
v_min = vertcat(*v_min)
v_max = vertcat(*v_max)

# Create an NLP solver
prob = {'f': J, 'x': V, 'g': g}
opts = {'ipopt.tol': 1e-3,
        'ipopt.max_iter': 100000
        }
solver = nlpsol('solver', 'ipopt', prob, opts)

# Solve the NLP
sol = solver(x0=v_init, lbx=v_min, ubx=v_max, lbg=g_min, ubg=g_max)
w_opt = sol['x'].full().flatten()

# Plot the solution
com_F_hist = Function("com_F_hist", [V], [comF_history])
com_F_hist_value = com_F_hist(w_opt).full()
com_B_hist = Function("com_B_hist", [V], [comB_history])
com_B_hist_value = com_B_hist(w_opt).full()
p_hist = Function("q_hist", [V], [p_history])
p_hist_value = p_hist(w_opt).full()
f_hist = Function("f_hist", [V], [f_history])
f_hist_value = f_hist(w_opt).full()


# PLOTS

FOOT_BR = p_hist_value[0:3]
FOOT_BL = p_hist_value[3:6]
FOOT_FR = p_hist_value[6:9]
FOOT_FL = p_hist_value[9:12]

blue = tuple(np.array([51., 51., 178.])/255.)
red = tuple(np.array([255., 0., 0.])/255.)
grey = tuple(np.array([192., 192., 192.])/255.)
purple = tuple(np.array([153., 0., 153.])/255.)
green = tuple(np.array([0., 255., 0.])/255.)
brown = tuple(np.array([204., 51., 0.])/255.)
orange = tuple(np.array([255., 153., 51.])/255.)
dark_green = tuple(np.array([0., 102., 0.])/255.)
pink = tuple(np.array([255., 153., 204.])/255.)
black = tuple(np.array([0., 0., 0.])/255.)

colors = [blue, red, grey, purple, green, brown, orange, dark_green, pink, black]


"""
fig1 = plt.figure()
ax = fig1.add_subplot(111, projection='3d')
ax.set_zlim(0.0, 0.6)
ax.set_xlim(-2.0, 4.0)
ax.set_ylim(-0.5, 1.5)
plt.pause(5)
for i in range(0, ns):
    COMF_i = [com_F_hist_value[0][i], com_F_hist_value[1][i], com_F_hist_value[2][i]]
    COMB_i = [com_B_hist_value[0][i], com_B_hist_value[1][i], com_B_hist_value[2][i]]
    plt.plot([COMF_i[0], COMB_i[0]], [COMF_i[1], COMB_i[1]], [COMF_i[2], COMB_i[2]], color = colors[i], linewidth=3)
    plt.plot([COMB_i[0], FOOT_BR[0, i]], [COMB_i[1], FOOT_BR[1, i]], [COMB_i[2], FOOT_BR[2, i]], color = colors[i], linewidth=3)
    plt.plot([COMB_i[0], FOOT_BL[0, i]], [COMB_i[1], FOOT_BL[1, i]], [COMB_i[2], FOOT_BL[2, i]], color = colors[i], linewidth=3)
    plt.plot([COMF_i[0], FOOT_FR[0, i]], [COMF_i[1], FOOT_FR[1, i]], [COMF_i[2], FOOT_FR[2, i]], color = colors[i], linewidth=3)
    plt.plot([COMF_i[0], FOOT_FL[0, i]], [COMF_i[1], FOOT_FL[1, i]], [COMF_i[2], FOOT_FL[2, i]], color = colors[i], linewidth=3)
    plt.pause(2)
plt.show()
"""
comF_final = comF_final.tolist()
comB_final = comB_final.tolist()

lin=np.linspace(-0.5, 2.5, 3000)
ones = np.ones(lin.shape)

x_gap_min = (x_min*ones).tolist()
x_gap_max = (x_max*ones).tolist()
x_safe_min = (gap_min*ones).tolist()
x_safe_max = (gap_max*ones).tolist()

y_gap = lin.tolist()
z_gap = np.zeros(lin.shape).tolist()

fig = plt.figure()
for i in range(0, ns):
    #ax = plt.axes()

    #ax = fig.add_subplot(111, projection='3d')
    ax = Axes3D(fig)
    
    title = 'Movement ' + str(i)
    ax.set_title(title)
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    ax.set_zlim(0.0, 1.5)
    ax.set_xlim(-1.0, 6.0)
    ax.set_ylim(-0.5, 2.5)

    ax.scatter([comF_final[0]], [comF_final[1]], [comF_final[2]], c = 'red', marker = '*', s=400, edgecolors=None)
    ax.scatter([comB_final[0]], [comB_final[1]], [comB_final[2]], c = 'red', marker = '*', s=400, edgecolors=None)
    COMF_i = [com_F_hist_value[0][i], com_F_hist_value[1][i], com_F_hist_value[2][i]]
    COMB_i = [com_B_hist_value[0][i], com_B_hist_value[1][i], com_B_hist_value[2][i]]
    plt.plot([COMF_i[0], COMB_i[0]], [COMF_i[1], COMB_i[1]], [COMF_i[2], COMB_i[2]], color = blue, linewidth=3)
    

    plt.plot([COMB_i[0], FOOT_BR[0, i]], [COMB_i[1], FOOT_BR[1, i]], [COMB_i[2], FOOT_BR[2, i]], color = blue, linewidth=3)
    plt.plot([COMB_i[0], FOOT_BL[0, i]], [COMB_i[1], FOOT_BL[1, i]], [COMB_i[2], FOOT_BL[2, i]], color = blue, linewidth=3)
    plt.plot([COMF_i[0], FOOT_FR[0, i]], [COMF_i[1], FOOT_FR[1, i]], [COMF_i[2], FOOT_FR[2, i]], color = blue, linewidth=3)
    plt.plot([COMF_i[0], FOOT_FL[0, i]], [COMF_i[1], FOOT_FL[1, i]], [COMF_i[2], FOOT_FL[2, i]], color = blue, linewidth=3)


    plt.plot([FOOT_BR[0, i], FOOT_BL[0, i]], [FOOT_BR[1, i], FOOT_BL[1, i]], [FOOT_BR[2, i], FOOT_BL[2, i]], color = black, linewidth = 1)
    plt.plot([FOOT_BL[0, i], FOOT_FL[0, i]], [FOOT_BL[1, i], FOOT_FL[1, i]], [FOOT_BL[2, i], FOOT_FL[2, i]], color = black, linewidth = 1)
    plt.plot([FOOT_FL[0, i], FOOT_FR[0, i]], [FOOT_FL[1, i], FOOT_FR[1, i]], [FOOT_FL[2, i], FOOT_FR[2, i]], color = black, linewidth = 1)
    plt.plot([FOOT_FR[0, i], FOOT_BR[0, i]], [FOOT_FR[1, i], FOOT_BR[1, i]], [FOOT_FR[2, i], FOOT_BR[2, i]], color = black, linewidth = 1)

    COM_i = ((mF*np.array(COMF_i) + mB*np.array(COMB_i))/(mF+mB)).tolist()
    plt.plot([COM_i[0], COM_i[0]], [COM_i[1], COM_i[1]], [COM_i[2], 0.], '--', color = black, linewidth = 1)
    ax.scatter([COM_i[0]], [COM_i[1]], [COM_i[2]], c = black, marker = 'o', s = 100)

    plt.plot(x_gap_min, y_gap, z_gap, color = black, linewidth = 1)
    plt.plot(x_gap_max, y_gap, z_gap, color = black, linewidth = 1)
    plt.plot(x_safe_min, y_gap, z_gap, '--', color = black, linewidth = 1)
    plt.plot(x_safe_max, y_gap, z_gap, '--', color = black, linewidth = 1)
    """
    if i != 0:
        COMF_im1 = [com_F_hist_value[0][i-1], com_F_hist_value[1][i-1], com_F_hist_value[2][i-1]]
        COMB_im1 = [com_B_hist_value[0][i-1], com_B_hist_value[1][i-1], com_B_hist_value[2][i-1]]
        plt.plot([COMF_im1[0], COMB_im1[0]], [COMF_im1[1], COMB_im1[1]], [COMF_im1[2], COMB_im1[2]], '--', color = blue, linewidth=2)

        plt.plot([COMB_im1[0], FOOT_BR[0, i-1]], [COMB_im1[1], FOOT_BR[1, i-1]], [COMB_im1[2], FOOT_BR[2, i-1]], '--', color = blue, linewidth=2)
        plt.plot([COMB_im1[0], FOOT_BL[0, i-1]], [COMB_im1[1], FOOT_BL[1, i-1]], [COMB_im1[2], FOOT_BL[2, i-1]], '--', color = blue, linewidth=2)
        plt.plot([COMF_im1[0], FOOT_FR[0, i-1]], [COMF_im1[1], FOOT_FR[1, i-1]], [COMF_im1[2], FOOT_FR[2, i-1]], '--', color = blue, linewidth=2)
        plt.plot([COMF_im1[0], FOOT_FL[0, i-1]], [COMF_im1[1], FOOT_FL[1, i-1]], [COMF_im1[2], FOOT_FL[2, i-1]], '--', color = blue, linewidth=2)
    """
    
    
    plt.pause(0.5)
    if i != ns-1:
        ax.remove()
plt.show()
    

"""

fig2 = plt.figure()
ax = plt.axes()
for i in range(1, ns):
    COMF_i = [com_F_hist_value[0][i], com_F_hist_value[1][i]]
    COMB_i = [com_B_hist_value[0][i], com_B_hist_value[1][i]]

    COMF_im1 = [com_F_hist_value[0][i-1], com_F_hist_value[1][i-1]]
    COMB_im1 = [com_B_hist_value[0][i-1], com_B_hist_value[1][i-1]]

    plt.plot([COMF_i[0], COMF_im1[0]],[COMF_i[1], COMF_im1[1]], '.-', color = blue, linewidth=2)
    plt.plot([COMB_i[0], COMB_im1[0]],[COMB_i[1], COMB_im1[1]], '.-', color = red, linewidth=2)

plt.show()
"""
