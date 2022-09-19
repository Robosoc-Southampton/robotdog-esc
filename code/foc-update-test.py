import testpid
from testpid import float_to_int as f_to_i
from math import sqrt

def velocity_pid_update(constants, minmax, target_v, v, v_prev_err, v_integral):
    target_q, v_integral = testpid.pid_update(constants, minmax, v - target_v, v_prev_err,
                                              v_integral)
    v_prev_err = v - target_v

    return target_q, v_integral, v_prev_err

v_pid_constants = [0.1, 0.1, 0.1]
v_pid_minmax = [-0.5, 0.5]
v_prev_err = 0
v_integral = 0

i_pid_constants = [0.1, 0.1, 0.1]
i_pid_minmax = [-0.5, 0.5]
id_prev_err = 0
iq_prev_err = 0
id_integral = 0
iq_integral = 0

def foc_update(ia, ib, sin_theta, cos_theta, v, target_v):
    global v_prev_err, v_integral, id_prev_err, iq_prev_err, id_integral, iq_integral
    target_q, v_integral, v_prev_err = velocity_pid_update(v_pid_constants, v_pid_minmax,
                                                           target_v, v, v_prev_err,
                                                           v_integral)
    target_d = 0
    
    ibeta = ib/sqrt(3) + ia/(2*sqrt(3))
    ialpha = ia/2

    i_q = ibeta*cos_theta - ialpha*sin_theta
    i_d = ibeta*sin_theta + ialpha*cos_theta
    
    vd, id_integral = testpid.pid_update(i_pid_constants, i_pid_minmax, i_d - target_d,
                                         id_prev_err, id_integral)
    id_prev_err = i_d - target_d

    vq, iq_integral = testpid.pid_update(i_pid_constants, i_pid_minmax, i_q - target_q,
                                         iq_prev_err, iq_integral)
    iq_prev_err = i_q - target_q

    valpha = vd*cos_theta - vq*sin_theta
    vbeta = vd*sin_theta + vq*cos_theta

    t_a = t_b = t_c = 0

    print("vd: {}  vq: {}  Valpha: {}  Vbeta: {}".format(vd, vq, valpha, vbeta))
    
    if vbeta > (-sqrt(3) * valpha) and vbeta > 0:
        t_a = valpha + vbeta/sqrt(3)
        t_b = 2*vbeta/sqrt(3)
    elif vbeta < (-sqrt(3)*valpha) and vbeta > sqrt(3)*valpha:
        t_b = -valpha + vbeta/sqrt(3)
        t_c = -valpha - vbeta/sqrt(3)
    else:
        t_a = valpha - vbeta/sqrt(3)
        t_c = -2*vbeta/sqrt(3)

    return t_a, t_b, t_c
    
trials = [
    [0, 0, 0, 1, 0.3, 0.3],
    [0, 0, 0, 1, 0, 0.3],
    [0, 0.2, 0, 1, 0, 0.3],
    [0, 0.2, 1, 0, 0, 0.3],
    [0.2, 0, 0, 1, 0, 0.3],
    [0.2, 0.2, 1, 0, 0, 0.3]
]

for t in trials:
    t_a, t_b, t_c = foc_update(t[0], t[1], t[2], t[3], t[4], t[5])
    print("t_a: {}  t_b: {}  t_c: {}".format(
        f_to_i(t_a), f_to_i(t_b), f_to_i(t_c)))
