with open('valoriPWM_verification.txt') as f:
    content = f.readlines()

with open('valoriPWM_verification.txt', 'w') as f:
    for line in content:
        f.write(line.rstrip() +'\n')
