import numpy as np
from scipy.integrate import odeint
from scipy.integrate import ode

def ComputeCommandParamsWithVelocity(pos_a_curr, v_a_curr, pos_t_curr, v_t, tf, t, kdrag):
    t0 = 0
    t1 = tf
    r0 = pos_a_curr
    r1 = pos_t_curr
    v0 = v_a_curr
    v1 = v_t

    if t is None:
        t = np.array([t0, t1])

    def dydt(t, y):
        return np.array([y[2], y[3], 4/(t1-t) * (v1 - y[2:4]) + 6/(t1-t)**2 * (r1 - (y[0:2] + v1*(t1 - t)))])

    r0v0 = np.concatenate((r0, v0))
    r = ode(dydt).set_integrator('dopri5').set_initial_value(r0v0, t0)
    t = [t0]
    r_list = [r0v0]

    while r.successful() and r.t < t1:
        r.integrate(t1, step=True)
        t = np.append(t, r.t)
        r_list.append(r.y[:4])

    r_list = np.array(r_list)
    r = r_list[:, :2]
    v = r_list[:, 2:4]
    a = np.zeros_like(v)
    norm2_a = np.zeros(len(t))

    for i in range(len(t)):
        a[i, :] = 4/(t1-t[i]) * (v1 - v[i, :]) + 6/(t1-t[i])**2 * (r1 - (r[i, :] + v1*(t1 - t[i])))
        norm2_a[i] = np.linalg.norm(a[i, :])**2

    a[np.isnan(a) | np.isinf(a)] = 0
    norm2_a[np.isnan(norm2_a) | np.isinf(norm2_a)] = 0

    rho = 1/2 * np.trapz(norm2_a, t)
    u = a

    return u, r, v, t, rho

