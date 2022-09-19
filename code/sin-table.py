import math

increments = 6000 >> 4
arr_name = "sin_lookup_tbl"

a = []

for i in range(increments):
    a.append(math.sin(math.pi*i/increments/2))


print("#include <stdfix.h>")
print("")
print("_Fract sin_lookup_tbl[{}] = {}".format(increments, "{"))

for x in a:
    print("    {:0.6}r,".format(x))

print("{};".format("}"))
