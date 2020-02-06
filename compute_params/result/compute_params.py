import os

label_path = "./labels/treated/"
output_path = "./outputs/per_image/"

label_files = os.listdir(label_path)
output_files = os.listdir(output_path)

label_files.sort()
output_files.sort()

if label_files != output_files:
    print("lists not equal! please run compare_files.py before!")
else:
    print("ready to compare")

for filename in label_files:
    #read lines in label
    lines_label = []
    label_f = open(label_path + filename, 'r')
    while True:
        line = label_f.readline() 
        if not line: 
            break
        split_line = line.split()
        lines_label.append(split_line)
    label_f.close() 

    #read lines in output
    lines_output = []
    output_f = open(output_path + filename, 'r')
    while True:
        line = output_f.readline() 
        if not line: 
            break
        split_line = line.split()
        lines_output.append(split_line)
    output_f.close() 

    #convert to int
    for line in lines_label:
        for nb in line:
            nb = int(nb)
    for line in lines_output:
        for nb in line:
            nb = int(nb)

    #compute centers 
    label_centers = []
    for line in lines_label:
        center = [(line[0]+line[2])/2, (line[1]+line[3])/2]
        label_centers.append(center)
    output_centers = []
    for line in lines_output:
        center = [(line[0]+line[2])/2, (line[1]+line[3])/2]
        output_centers.append(center)

    #todo solve hungarian algorithm, then compute params





















