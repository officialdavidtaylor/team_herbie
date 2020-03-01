# This script is intended to show what the code does to PS4 remote data

print("\nLeft Joystick\tRight Joystick\Left Wheel\Right Wheel\n")
for L in range(10):
    for R in range(10):

        rTemp = L + (.2 * (R)) # be careful not to divide by zero
        lTemp = L - (.2 * (R)) # "

        print((L-5)/5, (R-5)/5, lTemp, rTemp)
