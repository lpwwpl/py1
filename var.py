import time
import string

if __name__ == '__main__':
    file="D:/param.txt"
    with open(file, 'r') as f:
        while True:
            line = f.readline()
            if not line:
                break
            line = line.replace('\n', '').strip()
            list = line.split(' ')

            if len(list) == 7:
                str = "{}=[{},{},{},{},{},{}]".format(list[0],list[1],list[2],list[3],list[4],list[5],list[6])
                print(str)

