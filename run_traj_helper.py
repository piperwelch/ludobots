import os 
for i in range(500):
    os.system("python .\simulate.py DIRECT 0 {}".format(i))