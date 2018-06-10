# ENGR028 Homework 2 - Hyong Hark Lee - Bonus

R = 0.02
L = 0.1
B = 0.2

def radius(vr, vl):
  if vr == 0 and vl == 0:
    return -float('Inf')
  if vr == vl:
    return float('Inf')
  return abs((L/2) * (vr + vl) / (vr - vl))

def hitwall(wall, vr, vl):
  if wall > radius(vr, vl) + B:
    return False
  else:
    return True

def main():
  vr = float(input('What is the angular velocity of the right wheel? '))
  vl = float(input('How about the left? '))
  wall = float(input('Finally, how far is the wall? '))
  if hitwall(wall, vr, vl) == True:
    print('It will hit the wall. Good luck!')
  else:
    print('It will not hit the wall. Nice.')

if __name__ == "__main__":
  main()
