import os

width = 960
height = 540

original_path="./original"
treated_path="./treated"

original_files = os.listdir(original_path)

for original_file in original_files:
    f_original = open(original_path+"/"+original_file, 'r')
    f_treated = open(treated_path+"/"+original_file, 'a')
    while True:
        line = f_original.readline()
        if not line:
            break
        split_line = line.split()
        f_treated.write(str(int(width*(float(split_line[1])-float(split_line[3])/2)))+" "
                        +str(int(width*(float(split_line[2])-float(split_line[4])/2)))+" "
                        +str(int(width*(float(split_line[1])+float(split_line[3])/2)))+" "
                        +str(int(width*(float(split_line[2])+float(split_line[4])/2)))+"\n")
    f_original.close()
    f_treated.close()

