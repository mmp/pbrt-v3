import sys

print("Welcome")
sys.stdout.flush()

while True:
    sin = input()
    if sin == "a":
        print("child: aaa")
        sys.stdout.flush()
    if sin == "b":
        for i in range(3):
            print("child: bbb {}".format(i))
            sys.stdout.flush()
