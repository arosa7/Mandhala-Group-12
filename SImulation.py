import numpy as np
import magpylib as magpy
from scipy.optimize import minimize_scalar
#this import is for finding minimum values given an output especially within a range


M = 16  
r_inner = 0.024505  
dim = (0.003175, 0.003175, 0.019) 
pol = (1.4, 0, 0)
magnet_thickness = 0.003175  #width of magnet to calculate distance between the rings

# Calculate the target field we need to cancel
inner_ring = magpy.Collection()
for a in np.linspace(0, 360, M, endpoint=False):
    cube = magpy.magnet.Cuboid(dimension=dim, polarization=pol, position=(r_inner, 0, 0))
    cube.rotate_from_angax(a, 'z', anchor=0)
    cube.rotate_from_angax(a, 'z')
    inner_ring.add(cube)

sensor = magpy.Sensor(position=(0, 0, 0))
target_magnitude = np.linalg.norm(inner_ring.getB(sensor))
#creates the inner ring with halbach of 2 rotations (documentation had this, so I have just been rolling with it)
#since these magnets are rotated 2 times in their individual access it weakens the magnetic field in the inside which 
#is in our range of microtesla. 


print(f"--- ANCHOR RING (INNER) ---")
print(f"Magnets: {M}")
print(f"Target Center Field: {target_magnitude * 1000:.2f} mT\n")
#this is our target value based on the strength of the inner ring

min_gap_mm = 2.0
max_gap_mm = 15.0
#this is the range we can give so the code can find different distance between the rings 
#for optimal 3D printing
print(f"--- GENERATING VALID OUTER RING CONFIGURATIONS ---")
print(f"Looking for edge-to-edge gaps between {min_gap_mm} mm and {max_gap_mm} mm...")
print(f"{'Outer Magnets (N)':<18} | {'Exact Gap (mm)':<15} | {'Outer Radius (mm)':<18}")
print("-" * 55)

for N_test in range(M + 4, M + 40, 2):  # starts at 20 magnets and increments by 2, to keep # of magnets even

    def outer_field_guess(r_outer_guess):
        test_ring = magpy.Collection()
        for a in np.linspace(0, 360, N_test, endpoint=False):
            cube = magpy.magnet.Cuboid(dimension=dim, polarization=pol, position=(r_outer_guess, 0, 0))
            cube.rotate_from_angax(a, 'z', anchor=0)
            cube.rotate_from_angax(a, 'z')
            test_ring.add(cube)

        test_B = np.linalg.norm(test_ring.getB(sensor))
        return abs(test_B - target_magnitude) #guess can be less than target and it will throw a negative number
#the scipy function works by multiple guesses, this creates test rings 
#to identify best results withing our number of magnet range

    result = minimize_scalar(
        outer_field_guess, #no parantheses because this calls the function and it will run multiple times until it has a valid guess
        #these bounds make sure the rings dont overlap and that it does not get so huge
        bounds=(r_inner + magnet_thickness, r_inner + 0.05),
        method='bounded' #forces the function to remain within the bound
    )


    # If the optimizer found a perfect match
    if result.success and result.fun < 1e-5:
        #if the result is a success (valid radius found) and the function is less than 0.00001(almost perfect)
        r_outer_opt = result.x
        gap_m = r_outer_opt - r_inner - magnet_thickness
        gap_mm = gap_m * 1000
        #calculates gap between rings and converts to mm
        if min_gap_mm <= gap_mm <= max_gap_mm:
            print(f"{N_test:<18} | {gap_mm:<15.3f} | {r_outer_opt * 1000:<18.3f}")
            #just some formating i got from gemini, because mine was looking messy, nicer way to do this

print("-" * 55)

#this is some documentation I used to help me understand scipy. I was very confused on how to automate this
#so had to use other resources.
#https://docs.scipy.org/doc/scipy/reference/generated/scipy.optimize.minimize_scalar.html
