import numpy as np
from scipy import optimize

def initial_guess(alb: float):
    if alb > 0.56:
        ma = 1 - alb
        long_p = 1 - 0.4 * ma - 12 * (ma ** 2) / 175 - 2 * (ma ** 3) / 175 - 166 * (ma ** 4) / 67375
        k = np.sqrt(3 * ma) * long_p 
    else:
        exp = np.exp(-2 / alb)
        long_p = 1 + (4 - alb) / alb * exp + (exp ** 2) * (24 - 12 * alb + alb ** 2) / (alb ** 2) + \
            (exp ** 3) * (512 - 384 * alb + 72 * alb ** 2 - 3 * alb ** 3) / (alb ** 3)
        k = 1. - 2 * exp * long_p
    return 1 / k

if __name__ == "__main__":
    u_a = float(input("Sigma a (absorption): "))
    u_s = float(input("Sigma s (scattering): "))
    u_t = u_a + u_s
    albedo = u_s / u_t
    print(f"Sigma t = {u_t}, albedo = {albedo}")
    if albedo < 0.1:
        raise ValueError("Scattering albedo is too small. Unable to use this method.")
    if albedo > 0.9999:
        print("Warning: for albedo close to 1, solution does not exist. Falling back to use albedo = 0.9999")        
    ig = initial_guess(albedo)

    func2solve = lambda x: 1. - 0.5 * albedo * x * np.log((x + 1) / (x - 1))

    sol = optimize.fsolve(func2solve, ig)

    print(f"Solution to (albedo = {albedo:.5f}): {sol.item():.8f}, initial guess: {ig.item():.8f}, diff = {(sol - ig).item()}")

    

