import numpy as np
from scipy.integrate import quad

# Example function: f(x) = sin(x)
def f(x):
    return np.sin(x)

# Function to compute mean square over [a, b]
def mean_square(func, a, b):
    if a >= b:
        raise ValueError("Lower limit must be less than upper limit.")
    
    # Integrate the square of the function
    integral, error = quad(lambda x: func(x)**2, a, b)
    
    # Mean square value
    return integral / (b - a)

# Example usage
a, b = 0, np.pi  # Interval
ms_value = mean_square(f, a, b)

print(f"Mean square of f(x) over [{a}, {b}] is: {ms_value:.6f}")