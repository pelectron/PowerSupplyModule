import fractions
from fixedpoint import FixedPoint
from decimal import Decimal
import math
import numpy as np

for i in range(64):
    for k in range(31, 0, -1):
        v = 2**i * 10 ** (k)
        s = str(v)
        if v < 2**64:
            print(f"case {i}: return {k};")
            break
exit()
print("least significant names")
for i in range(1, 64):
    s = f"{FixedPoint(2**-i, n=i):.63f}"[2:]
    r = s[::-1]
    count = 0
    while r[count] == "0":
        count = count + 1
    r = r[count:]
    s = r[::-1]

    count = 0
    while s[count] == "0":
        count = count + 1
    s = s[count:]
    d = int(s)
    if d < 2**64:
        rem = d % int(2**32)
        n = int(d / int(2**32)) % int(2**32)
        if True:
            # print(f"case {i}: return u64{{ {rem}u, {n}u }};")
            # print(f"case {i}: return  {len(s)}u;")
            print(
                f"case {i}: return  {len(str(FixedPoint(2**-i, n=i) * (2**i - 1) * 10**12))}u;"
            )
        else:
            print(f"case {i}: {int(np.log(d) / np.log(10))}, {len(s)}")

print(25243548967072379233715559038189568 > (1 << 64))
