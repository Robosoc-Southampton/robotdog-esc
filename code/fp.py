
def fp_1_15_to_int(fp115):
    bit = 0
    val = 0
    for c in fp115:
        bitval = 2**(-bit) if bit > 0 else -1
        if c == "1":
            val += bitval
        bit += 1
    return val

def int_to_fp_1_15(val):
    fp115 = "1" if val < 0 else "0"
    curr = -1 if val < 0 else 0
    for i in range(1,16):
        bitval = 2**(-i)
        if (curr + bitval) <= val:
            fp115 += "1"
            curr += bitval
        else:
            fp115 += "0"
    return fp115

        
    
# "{:>016}".format(bin())
