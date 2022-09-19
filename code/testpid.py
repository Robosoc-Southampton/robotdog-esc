import fp

pid_constants = [0.1, 0.1, 0.1]
pid_minmax = [-0.5, 0.5]

trials = [
    [0, 0, 0],
    [0.3, 0.3, 0],
    [0.3, 0, 0],
    [0.3, 0, 0.2],
    [0.3, 0, -0.2],
    [0.9, 0.6, 0],
    [0.9, 0.6, -0.1],
    [0, 0, 0.6],
    [0, 0, -0.6]
]

def pid_update(constants, minmax, curr_err, prev_err, integral):
    integral += curr_err*constants[1]
    x = integral + curr_err*constants[0] + (curr_err-prev_err)*constants[1]
    if x > minmax[1]:
        integral -= x - minmax[1]
    if x < minmax[0]:
        integral += minmax[0] - x
    x = integral + curr_err*constants[0] + (curr_err-prev_err)*constants[1]
    return x, integral

def float_to_int(f):
    x = int(f * 32768)
    if x < 0:
        x += 65536
    return x
    

"""
for t in trials:
    x, i = pid_update(pid_constants, pid_minmax, t[0], t[1], t[2])
    print("Output: {}  Integral: {}".format(
        float_to_int(x), float_to_int(i)))
"""
