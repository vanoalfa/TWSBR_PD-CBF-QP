# Source - https://stackoverflow.com/a/50108215
# Posted by eyllanesc, modified by community. See post 'Timeline' for change history
# Retrieved 2026-09-14, License - CC BY-SA 3.0

from scipy.signal import *
from scipy.integrate import quad
from numpy import *
p = 10
na = arange(0,10,1)
y = []

def integrand(t, a, b):
    return square(t)*cos(b*((2*pi)/a)*t)

for e in na:
    i,err = quad(integrand,0,p, args=(p, e))
    y.append(i)

print(y)
