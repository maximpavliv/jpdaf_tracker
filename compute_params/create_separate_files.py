import os

threshold = 0.5

rescale = 0.5

file_everything = open('comp4_det_test_drone.txt', 'r')
count = 0

lines = []

while True:
    count += 1

    # Get next line from file 
    line = file_everything.readline() 
    split_line = line.split()
    # if line is empty 
    # end of file is reached 
    if not line: 
        break
    #print("Line{}: {}".format(count, line.strip())) 
    lines.append(split_line)
file_everything.close() 

print("nb_lines: " + str(len(lines)))

for line in lines:
    filename = "./per_image/"+line[0]+".txt"
    f=open(filename, "a")
    if float(line[1]) >= threshold:
        f.write(str(int(float(line[2])*rescale))+" "+str(int(float(line[3])*rescale))+" "+str(int(float(line[4])*rescale))+" "+str(int(float(line[5])*rescale))+"\n")
    f.close()
        
    
