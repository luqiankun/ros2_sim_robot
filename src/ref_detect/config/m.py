

f = open('map_post.txt')

line = f.readline()
fs = open("t.txt", 'a')
while line != '':
    print(line)
    line = line.replace("\n", "")
    str = line.split('|')
    # print(str)
    ns = list(map(float, str))
    # print(ns[0])
    res = "R{:d}=({:f},{:f})".format(int(ns[0]), ns[1], ns[2])
    print(res)
    fs.write(res+"\n")
    line = f.readline()
f.close()
fs.close()
