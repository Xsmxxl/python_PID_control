from scipy.integrate import quad

def integrand(x):
    return x**2

print(quad(integrand, 0, 1))