import random
import sys

# adapt the number of random floats you want printed here
config_count = 3

def rndSign():
    if (random.random() < 0.5 ):
        return ""
    else :
        return "-"

sys.stdout.write(rndSign() + str(random.random()))
for i in range(config_count-1):
    sys.stdout.write(" " + rndSign() + str(random.random()))

#[random.random() for i in range(9)]