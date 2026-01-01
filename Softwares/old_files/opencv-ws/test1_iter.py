import math
from itertools import combinations

points = [(1, 1), (2, 2), (3, 3), (4, 4), (5, 5)]

arr = []

for p in points:

    # print(p)

    pp = [x for x in points if x != p]

    for combo in combinations(pp, 2):
        combo = (p,) + combo
        arr.append(combo)

print(arr)
print(len(arr))


