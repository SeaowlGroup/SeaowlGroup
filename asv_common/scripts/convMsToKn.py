import sys

#Utils
if __name__ == "__main__":
    if len(sys.argv) <= 1:
        print(f'Usage: {sys.argv[0]} <file name> [mtk/ktm]')
        sys.exit(1)
    else:
        serial = sys.argv[1]
        if len(sys.argv)==3:
            mtk = (sys.argv[2] == 'mtk')
        else:
            mtk = True
    f1 = open(f'{serial}.txt', 'r')
    f2 = open(f'{serial}_conv.txt', 'w')
    lines = f1.readlines()
    for line in lines:
        content = line.split()
        if mtk:
            u_d_asv = round(float(content[2])*1.94384)
            u_d = round(float(content[5])*1.94384)
        else:
            u_d_asv = round(float(content[2])/1.94384)
            u_d = round(float(content[5])/1.94384)
        for k in range(len(content)):
            if k == 2:
                f2.write(f'{u_d_asv}    ')
            elif k== 5:
                f2.write(f'{u_d}    ')
            else:
                f2.write(f'{content[k]}    ')
        f2.write(f'\n')
    f1.close()
    f2.close()
    