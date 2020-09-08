#!/usr/bin/env python
from casadi import *


def gait_generation(p_init, moving_foot, step_counter, SP_center_last, com_last, pos_file, vel_file, forces_file):
    #### PROBLEM DATA ####
    m = 92.         # total mass of the robot
    n_p = 4         # number of feet

    dim_com = 3     # dimension of com coordinate-vector 
    dim_p = 3*n_p   # dimension of the stack of feet coordinate-vectors [FL-FR-BL-BR] (on file [BR-BL-FR-FL])
    dim_n = 3*n_p   # dimension of the stack of normal vector to the ground
    dim_f = 3*n_p   # dimension of the stack of contact forces

    step_length = 0.2

    g = 9.81
    weight = np.array([0., 0., -m*g])

    T_1 = 1.       # the CoM has been moved keeping the stance
    T_2 = 3.       # the step is completed
    T = 4.         # time horizon

    sampling_time = 0.1

    n_s = int(T/sampling_time) # number of samples

    # Contacts and normals
    p_FL = np.hstack((p_init[6:8], 0.))
    p_FR = np.hstack((p_init[4:6], 0.))
    p_BL = np.hstack((p_init[2:4], 0.))
    p_BR = np.hstack((p_init[0:2], 0.))
    
    n_FL = np.array([0., 0., 1.])
    n_FR = np.array([0., 0., 1.])
    n_BL = np.array([0., 0., 1.])
    n_BR = np.array([0., 0., 1.])

    # Initial conditions
    if step_counter == 0:
        x_center = (p_FL[0] + p_FR[0] + p_BL[0] + p_BR[0])/4
        y_center = (p_FL[1] + p_FR[1] + p_BL[1] + p_BR[1])/4     
        com_init = np.array([x_center, y_center, 0.]) + np.array([0.108, 0., 0.63]) # np.array([0.143, 0., 0.620])
    elif step_counter == 1 or step_counter == 3:
        com_init = com_last
    elif step_counter == 2:
        x_center = (p_FL[0] + p_FR[0] + p_BL[0] + p_BR[0])/4
        y_center = (p_FL[1] + p_FR[1] + p_BL[1] + p_BR[1])/4
        z_center = 0.
        SP_center = np.array([x_center, y_center, z_center])
        delta = com_last - SP_center_last
        com_init = SP_center + delta

    if step_counter == 0:
        p_BR[0] += 0.2
        p_BL[0] += 0.2

    dcom_init = np.array([0., 0., 0.])
    dcom_final = np.array([0., 0., 0.])
    f_init = np.zeros((12, 1))

    
    # Bounds (in order to avoid that the com remains behind too much, xcom_max for the last step is larger ---> the pelvis can go forward)
    com_min = np.array([com_init[0]-0.05, com_init[1]-0.12, 0.55])
    if step_counter == 3:
        com_max = np.array([com_init[0]+0.15, com_init[1]+0.12, 0.64])
    else:
        com_max = np.array([com_init[0]+0.05, com_init[1]+0.12, 0.64])
    dcom_min = 2*np.array([[-0.15, -0.15, -0.15]])
    dcom_max = 2*np.array([0.15, 0.15, 0.15])
    f_min = np.array([-1000., -1000., 0., -1000., -1000., 0., -1000., -1000., 0., -1000., -1000., 0.])
    f_max = np.array([1000., 1000., 1000., 1000., 1000., 1000., 1000., 1000., 1000., 1000., 1000., 1000.])        
    

    # Symbolic variables
    com = SX.sym('com', dim_com)
    dcom = SX.sym('dcom', dim_com)
    x = vertcat(com, dcom)

    p = SX.sym('p', dim_p)
    n = SX.sym('n', dim_n)
    f = SX.sym('f', dim_f)

    point = SX.sym('point', 3)
    normal = SX.sym('normal', 3)
    force = SX.sym('force', 3)

    tau = SX.sym('tau', 3)


    # DYNAMICAL CONSTRAINT
    # Dynamic equation
    g_vec = SX([0., 0., -g])
    xdot = vertcat(x[3:6], g_vec + (f[0:3] + f[3:6] + f[6:9] + f[9:12])/m)


    # Lagrangian (minimize com velocity, forces, zcom and: a) step 1-3: com traveled distance (xy); b) step 4: distances com-pFL & com-pFR ---> the pelvis goes forward)
    l_p = 1000.
    l_v = 10.
    l_f = 0.1#100.
    l_z = 10000.
    l_fin = 100000.

    if step_counter == 3:
        L = l_v * dot(dcom, dcom) + l_f * dot(f, f) + l_z * dot(com[2], com[2]) + l_fin * dot(com - p_FL, com - p_FL) + l_fin * dot(com - p_FR, com - p_FR)
    else:
        L = l_p * dot(com[0:2]-com_init[0:2], com[0:2] - com_init[0:2]) + l_v * dot(dcom, dcom) + l_f * dot(f, f) + l_z * dot(com[2], com[2])

    # Discrete time dynamics
    ode = {'x':x, 'p':f, 'ode':xdot, 'quad':L}
    opts = {'tf':sampling_time}
    F_dy = integrator('F_dy', 'cvodes', ode, opts)
    '''
    # Fixed step Runge-Kutta 4 integrator
    M = 4 # RK4 steps per interval
    DT = T/n_s/M
    f_dy = Function('f', [x, f], [xdot, L])
    X0 = MX.sym('X0', 6)
    U = MX.sym('U', 12)
    X = X0
    Q = 0
    for j in range(M):
       k1, k1_q = f_dy(X, U)
       k2, k2_q = f_dy(X + DT/2 * k1, U)
       k3, k3_q = f_dy(X + DT/2 * k2, U)
       k4, k4_q = f_dy(X + DT * k3, U)
       X=X+DT/6*(k1 +2*k2 +2*k3 +k4)
       Q = Q + DT/6*(k1_q + 2*k2_q + 2*k3_q + k4_q)
    F_dy = Function('F', [X0, U], [X, Q],['x0','p'],['xf','qf'])
    '''

    # STATIC CONSTRAINTS
    # No rotation
    delta = vertcat(com, com ,com, com) - p[0:12]
    S1 = skew(delta[0:3])
    S2 = skew(delta[3:6])
    S3 = skew(delta[6:9])
    S4 = skew(delta[9:12])
    Scat = horzcat(S1, S2, S3, S4)
    tau = mtimes(Scat, f)

    F_rot = Function('F_rot', [com, p, f], [tau], ['com', 'p', 'f'], ['tau'])

    # Friction cones (actually friction pyramids)
    friction_cone = SX.sym('friction_cone', 2)

    F_thr = 0.0
    mi = 1.50

    mi_tilde = mi/sqrt(2)

    friction_cone0 = -force[2]
    friction_cone1 = force[0] - mi_tilde * force[2]
    friction_cone2 = -force[0] - mi_tilde * force[2]
    friction_cone3 = force[1] - mi_tilde * force[2]
    friction_cone4 = -force[1] - mi_tilde * force[2]

    friction_cone = vertcat(friction_cone0, friction_cone1, friction_cone2, friction_cone3, friction_cone4)

    F_fr = Function('F_fr', [force, normal], [friction_cone], ['force', 'normal'], ['friction_cone'])


    # Start with an empty NLP
    NV = (2*dim_com + dim_f) * n_s + 2*dim_com
    V = MX.sym('V', NV)

    # NLP vars bounds and initial conditions
    v_min = []
    v_max = []
    v_init = []
    g_min = []
    g_max = []

    offset = 0

    COM = []
    DCOM = []
    F = []

    # Formulate the NLP
    for k in range(n_s+1):
        # COM at the k-th time sample
        COM.append(V[offset:offset+dim_com])
        
        if k == 0:
            v_min += com_init.tolist()
            v_max += com_init.tolist()
        else:
            v_min += com_min.tolist()
            v_max += com_max.tolist()

        v_init += com_init.tolist()

        offset += dim_com

        # DCOM at the k-th time sample
        DCOM.append(V[offset:offset+dim_com])
        
        if k == 0:
            v_min += dcom_init.tolist()
            v_max += dcom_init.tolist()
        elif k == n_s:
            v_min += dcom_final.tolist()
            v_max += dcom_final.tolist()
        else:
            v_min += dcom_min.tolist()
            v_max += dcom_max.tolist()

        v_init += dcom_init.tolist()

        offset += dim_com

        # Forces at k-th time sample
        if k<n_s:
            F.append(V[offset:offset + dim_f])

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

    com_history = MX(Sparsity.dense(dim_com, n_s+1))
    dcom_history = MX(Sparsity.dense(dim_com, n_s+1))
    f_history = MX(Sparsity.dense(dim_f, n_s))


    for k in range(n_s):
        
        COM_k = COM[k]
        DCOM_k = DCOM[k]
        
        
        F_k = F[k]
        F_FL_k = F_k[0:3]
        F_FR_k = F_k[3:6]
        F_BL_k = F_k[6:9]
        F_BR_k = F_k[9:12]

        if moving_foot == 'FL':
            if k < int(T_1/sampling_time):
                P_FL_k = p_FL
            elif k >= int(T_1/sampling_time) and k <= int(T_2/sampling_time):
                t = k*sampling_time
                x = step_length/2 - step_length/2 * cos(2*pi*(t-T_1)/(2*(T_2 - T_1))) + p_FL[0]
                y = p_FL[1]
                z = step_length/2 * sin(2*pi*(t-T_1)/(2*(T_2 - T_1))) + p_FL[2]
                P_FL_k = np.array([x, y, z])
            else:
                P_FL_k = p_FL + np.array([step_length, 0., 0.])   
            P_FR_k = p_FR
            P_BL_k = p_BL
            P_BR_k = p_BR
            
        elif moving_foot == 'FR':
            P_FL_k = p_FL
            if k < int(T_1/sampling_time):
                P_FR_k = p_FR
            elif k >= int(T_1/sampling_time) and k <= int(T_2/sampling_time):
                t = k*sampling_time
                x = step_length/2 - step_length/2 * cos(2*pi*(t-T_1)/(2*(T_2 - T_1))) + p_FR[0]
                y = p_FR[1]
                z = step_length/2 * sin(2*pi*(t-T_1)/(2*(T_2 - T_1))) + p_FR[2]
                P_FR_k = np.array([x, y, z])
            else:
                P_FR_k = p_FR + np.array([step_length, 0., 0.])   
            P_BL_k = p_BL
            P_BR_k = p_BR

        elif moving_foot == 'BL':
            P_FL_k = p_FL
            P_FR_k = p_FR
            if k <= int(T_2/sampling_time):
                P_BL_k = p_BL
            else:
                P_BL_k = p_BL + np.array([step_length, 0., 0.]) 
            P_BR_k = p_BR
            
        elif moving_foot == 'BR':
            P_FL_k = p_FL
            P_FR_k = p_FR
            P_BL_k = p_BL
            if k <= int(T_2/sampling_time):
                P_BR_k = p_BR
            else:
                P_BR_k = p_BR + np.array([step_length, 0., 0.])               

        n_FL_k = n_FL
        n_FR_k = n_FR
        n_BL_k = n_BL
        n_BR_k = n_BR

        
        F_dy_k = F_dy(x0 = vertcat(COM_k, DCOM_k), p=F_k)
        
        # Cost function
        J += F_dy_k['qf']

        # CONSTRAINTS
        # Continuity condition: f(x_{k}, u_{k}) = x_{k+1}
        COM_kp1 = COM[k+1]
        DCOM_kp1 = DCOM[k+1]

        g += [vertcat(COM_kp1, DCOM_kp1) - F_dy_k['xf']]
        g_min += np.zeros((6, 1)).tolist()
        g_max += np.zeros((6, 1)).tolist()
        
        # No rotations: tau_{k} = 0
        tau_k = F_rot(com = COM_k, p = vertcat(P_FL_k, P_FR_k, P_BL_k, P_BR_k), f = F_k)['tau']    
        g += [tau_k]

        g_min += np.array([-1., -1., -1.]).tolist()
        g_max += np.array([1., 1., 1.]).tolist()

        
        # Friction cones
        if moving_foot == 'FL':
            fr_FR_k = F_fr(force=F_FR_k, normal=n_FL_k)['friction_cone']
            fr_BL_k = F_fr(force=F_BL_k, normal=n_FL_k)['friction_cone']
            fr_BR_k = F_fr(force=F_BR_k, normal=n_FL_k)['friction_cone']
            if (k >= 0 and k < int(T_1/sampling_time)) or k > int(T_2/sampling_time):
                fr_FL_k = F_fr(force=F_FL_k, normal=n_FL_k)['friction_cone']
                g += [fr_FL_k, fr_FR_k, fr_BL_k, fr_BR_k]
                g_min += np.array([-1000., -1000., -1000., -1000., -1000., -1000., -1000., -1000., -1000., -1000., -1000., -1000., -1000., -1000., -1000., -1000., -1000., -1000., -1000., -1000.]).tolist()
                g_max += np.zeros((20, 1)).tolist()
            elif k >= int(T_1/sampling_time) and k <= int(T_2/sampling_time):
                g += [fr_FR_k, fr_BL_k, fr_BR_k]
                g_min += np.array([-1000., -1000., -1000., -1000., -1000., -1000., -1000., -1000., -1000., -1000., -1000., -1000., -1000., -1000., -1000.]).tolist()
                g_max += np.zeros((15, 1)).tolist()

        elif moving_foot == 'FR':
            fr_FL_k = F_fr(force=F_FL_k, normal=n_FL_k)['friction_cone']
            fr_BL_k = F_fr(force=F_BL_k, normal=n_FL_k)['friction_cone']
            fr_BR_k = F_fr(force=F_BR_k, normal=n_FL_k)['friction_cone']
            if (k >= 0 and k < int(T_1/sampling_time)) or k > int(T_2/sampling_time):
                fr_FR_k = F_fr(force=F_FR_k, normal=n_FL_k)['friction_cone']
                g += [fr_FL_k, fr_FR_k, fr_BL_k, fr_BR_k]
                g_min += np.array([-1000., -1000., -1000., -1000., -1000., -1000., -1000., -1000., -1000., -1000., -1000., -1000., -1000., -1000., -1000., -1000., -1000., -1000., -1000., -1000.]).tolist()
                g_max += np.zeros((20, 1)).tolist()
            elif k >= int(T_1/sampling_time) and k <= int(T_2/sampling_time):
                g += [fr_FL_k, fr_BL_k, fr_BR_k]
                g_min += np.array([-1000., -1000., -1000., -1000., -1000., -1000., -1000., -1000., -1000., -1000., -1000., -1000., -1000., -1000., -1000.]).tolist()
                g_max += np.zeros((15, 1)).tolist()

        elif moving_foot == 'BL':
            fr_FL_k = F_fr(force=F_FL_k, normal=n_FL_k)['friction_cone']
            fr_FR_k = F_fr(force=F_FR_k, normal=n_FL_k)['friction_cone']
            fr_BR_k = F_fr(force=F_BR_k, normal=n_FL_k)['friction_cone']
            if (k >= 0 and k < int(T_1/sampling_time)) or k > int(T_2/sampling_time):
                fr_BL_k = F_fr(force=F_BL_k, normal=n_FL_k)['friction_cone']
                g += [fr_FL_k, fr_FR_k, fr_BL_k, fr_BR_k]
                g_min += np.array([-1000., -1000., -1000., -1000., -1000., -1000., -1000., -1000., -1000., -1000., -1000., -1000., -1000., -1000., -1000., -1000., -1000., -1000., -1000., -1000.]).tolist()
                g_max += np.zeros((20, 1)).tolist()
            elif k >= int(T_1/sampling_time) and k <= int(T_2/sampling_time):
                g += [fr_FL_k, fr_FR_k, fr_BR_k]
                g_min += np.array([-1000., -1000., -1000., -1000., -1000., -1000., -1000., -1000., -1000., -1000., -1000., -1000., -1000., -1000., -1000.]).tolist()
                g_max += np.zeros((15, 1)).tolist()

        if moving_foot == 'BR':
            fr_FL_k = F_fr(force=F_FL_k, normal=n_FL_k)['friction_cone']
            fr_FR_k = F_fr(force=F_FR_k, normal=n_FL_k)['friction_cone']
            fr_BL_k = F_fr(force=F_BL_k, normal=n_FL_k)['friction_cone']
            if (k >= 0 and k < int(T_1/sampling_time)) or k > int(T_2/sampling_time):
                fr_BR_k = F_fr(force=F_BR_k, normal=n_FL_k)['friction_cone']
                g += [fr_FL_k, fr_FR_k, fr_BL_k, fr_BR_k]
                g_min += np.array([-1000., -1000., -1000., -1000., -1000., -1000., -1000., -1000., -1000., -1000., -1000., -1000., -1000., -1000., -1000., -1000., -1000., -1000., -1000., -1000.]).tolist()
                g_max += np.zeros((20, 1)).tolist()
            elif k >= int(T_1/sampling_time) and k <= int(T_2/sampling_time):
                g += [fr_FL_k, fr_FR_k, fr_BL_k]
                g_min += np.array([-1000., -1000., -1000., -1000., -1000., -1000., -1000., -1000., -1000., -1000., -1000., -1000., -1000., -1000., -1000.]).tolist()
                g_max += np.zeros((15, 1)).tolist()
        
        # Zero force on swinging foot
        if k > int(T_1/sampling_time) and k < int(T_2/sampling_time):
            if moving_foot == 'FL':
                g += [F_FL_k]
                g_min += np.zeros((3, 1)).tolist()
                g_max += np.zeros((3, 1)).tolist()
            
            elif moving_foot == 'FR':
                g += [F_FR_k]
                g_min += np.zeros((3, 1)).tolist()
                g_max += np.zeros((3, 1)).tolist()

            elif moving_foot == 'BL':
                g += [F_BL_k]
                g_min += np.zeros((3, 1)).tolist()
                g_max += np.zeros((3, 1)).tolist()
            
            elif moving_foot == 'BR':
                g += [F_BR_k]
                g_min += np.zeros((3, 1)).tolist()
                g_max += np.zeros((3, 1)).tolist()
    

        #save results
        com_history[0:dim_com, k] = COM_k
        dcom_history[0:dim_com, k] = DCOM_k
        f_history[0:dim_f, k] = F_k


    com_history[0:dim_com, k+1] = COM[k+1]
    dcom_history[0:dim_com, k+1] = DCOM[k+1]   

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
    print('------------------------------------------------------------------------------------------------')
    w_opt = sol['x'].full().flatten()
    
    # Plot the solution
    com_hist = Function("com_hist", [V], [com_history])
    com_hist_value = com_hist(w_opt).full()
    dcom_hist = Function("dcom_hist", [V], [dcom_history])
    dcom_hist_value = dcom_hist(w_opt).full()
    f_hist = Function("f_hist", [V], [f_history])
    f_hist_value = f_hist(w_opt).full()
    '''    
    tplot = [T/n_s*k for k in range(n_s+1)]
    import matplotlib.pyplot as plt
    plt.figure(1)
    plt.clf()
    plt.plot(tplot, com_hist_value[0],tplot, com_hist_value[1], tplot, com_hist_value[2], '--')
    plt.xlabel('t')
    plt.ylabel('p_com')
    plt.legend(['x', 'y', 'z'])
    plt.grid()

    plt.figure(2)
    plt.clf()
    plt.plot(tplot, dcom_hist_value[0],tplot, dcom_hist_value[1], tplot, dcom_hist_value[2], '--')
    plt.xlabel('t')
    plt.ylabel('dp_com')
    plt.legend(['dx', 'dy', 'dz'])
    plt.grid()

    plt.figure(3)
    plt.clf()
    plt.plot(com_hist_value[1], com_hist_value[0], '--')
    plt.ylabel('B------------------------F')
    plt.xlabel('L------------------------R')
    plt.gca().invert_xaxis()
    plt.grid()

    plt.figure(4)
    plt.clf()
    for i in range(0, 12, 3):
        plt.step(tplot, vertcat(DM.nan(1),f_hist_value[i]), linewidth = 12-i)
    plt.xlabel('t')
    plt.ylabel('fx_c')
    plt.legend(['fx_FL', 'fx_FR', 'fx_BL', 'fx_BR'])
    plt.grid()

    plt.figure(5)
    plt.clf()
    for i in range(1, 12, 3):
        plt.step(tplot, vertcat(DM.nan(1),f_hist_value[i]), linewidth = 12-i)
    plt.xlabel('t')
    plt.ylabel('fy_c')
    plt.legend(['fy_FL', 'fy_FR', 'fy_BL', 'fy_BR'])
    plt.grid()

    plt.figure(6)
    plt.clf()
    for i in range(2, 12, 3):
        plt.step(tplot, vertcat(DM.nan(1),f_hist_value[i]), linewidth = 12-i)
    plt.xlabel('t')
    plt.ylabel('fz_c')
    plt.legend(['fz_FL', 'fz_FR', 'fz_BL', 'fz_BR'])
    plt.grid()

    plt.show()
    '''
    
    # Write solution on file
    for i in range(0, n_s+1):
        pos_file.write(str(com_hist_value[0, i]) + " " + str(com_hist_value[1, i]) + " " + str(com_hist_value[2, i]) + "\n")
        vel_file.write(str(dcom_hist_value[0, i]) + " " + str(dcom_hist_value[1, i]) + " " + str(dcom_hist_value[2, i]) + "\n")
        if i < n_s:
            forces_file.write(str(f_hist_value[0, i]) + " " + str(f_hist_value[1, i]) + " " + str(f_hist_value[2, i]) + " " + str(f_hist_value[3, i]) + " " + str(f_hist_value[4, i]) + " " + str(f_hist_value[5, i]) + " " + str(f_hist_value[6, i]) + " " + str(f_hist_value[7, i]) + " " + str(f_hist_value[8, i]) + " " + str(f_hist_value[9, i]) + " " + str(f_hist_value[10, i]) + " " + str(f_hist_value[11, i]) + "\n")

    SP_center = (P_FL_k + P_FR_k + P_BL_k + P_BR_k)/4
    com_last = com_hist_value[0:3, -1]
    return SP_center, com_last





def main():
    data = np.loadtxt('path_example.txt')
    dim = len(data[:, 0])
    
    step_counter = 0
    SP_center_last = np.zeros((3, 1))
    com_last = np.zeros((3, 1))
    
    for i in range(dim):
        primitive = data[i, -2]
        
        if primitive == 9:                
            moving_foot = 'FL'
            p_init = data[i-1, 0:8]
            pos_file = open(str(step_counter)+'_COMtraj_FL.txt','w')
            vel_file = open(str(step_counter)+'_DCOMtraj_FL.txt','w')
            forces_file = open(str(step_counter)+'_forces_FL.txt','w')
            SP_center_last, com_last = gait_generation(p_init, moving_foot, step_counter, SP_center_last, com_last, pos_file, vel_file, forces_file)
            step_counter += 1
           
        elif primitive == 8:
            moving_foot = 'FR'
            pos_file = open(str(step_counter)+'_COMtraj_FR.txt','w')
            vel_file = open(str(step_counter)+'_DCOMtraj_FR.txt','w')
            forces_file = open(str(step_counter)+'_forces_FR.txt','w')
            p_init = data[i-1, 0:8]
            SP_center_last, com_last = gait_generation(p_init, moving_foot, step_counter, SP_center_last, com_last, pos_file, vel_file, forces_file)
            step_counter += 1
        
        elif primitive == 7:
            moving_foot = 'BL'
            pos_file = open(str(step_counter)+'_COMtraj_BL.txt','w')
            vel_file = open(str(step_counter)+'_DCOMtraj_BL.txt','w')
            force_file = open(str(step_counter)+'_forces_BL.txt','w')
            p_init = data[i-1, 0:8]
            SP_center_last, com_last = gait_generation(p_init, moving_foot, step_counter, SP_center_last, com_last, pos_file, vel_file, forces_file)
            step_counter += 1
            
        elif primitive == 6:
            moving_foot = 'BR'
            p_init = data[i-1, 0:8]
            pos_file = open(str(step_counter)+'_COMtraj_BR.txt','w')
            vel_file = open(str(step_counter)+'_DCOMtraj_BR.txt','w')
            forces_file = open(str(step_counter)+'_forces_BR.txt','w')
            SP_center_last, com_last = gait_generation(p_init, moving_foot, step_counter, SP_center_last, com_last, pos_file, vel_file, forces_file)
            step_counter += 1
        

if __name__ == "__main__":
    com_hist_value = main()   
        
















