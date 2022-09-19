def fixed_to_float(x):
    if x >= (2 ** 15):
        return (x / (2**15)) - 2
    else:
        return x / (2**15)

def calc_vals(w4, w5, w6, w7):
    sintheta = fixed_to_float(w4)
    ibeta = fixed_to_float(w5)
    ialpha = fixed_to_float(w6)
    costheta = fixed_to_float(w7)

    print("ibcos: {:.6}  ibsin: {:.6}  iacos: {:.6}  iasin: {:.6}".format(
        ibeta*costheta, ibeta*sintheta, ialpha*costheta, ialpha*sintheta))
    
