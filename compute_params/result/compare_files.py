import os

create_missing_files_in_p2 = False

path1 = "./labels/treated/"
path2 = "./outputs/per_image/"

files_path1 = os.listdir(path1)
files_path2 = os.listdir(path2)

unexistent_files_in_path1 = []
unexistent_files_in_path2 = []

for file_path1 in files_path1:
    if not os.path.isfile(path2+file_path1):
        unexistent_files_in_path2.append(file_path1)

for file_path2 in files_path2:
    if not os.path.isfile(path1+file_path2):
        unexistent_files_in_path1.append(file_path2)

print("Unexistent files in path 1: ")
for unex_file in unexistent_files_in_path1:
    print(unex_file)
print("--------------------------------------------")
print("Unexistent files in path 2: ")
for unex_file in unexistent_files_in_path2:
    print(unex_file)
print("--------------------------------------------")
print("Nb of files in path 1: "+str(len(files_path1)))
print("Nb of files in path 2: "+str(len(files_path2)))
print("Nb of files existing in path 1 but not 2: "+str(len(unexistent_files_in_path2)))
print("Nb of files existing in path 2 but not 1: "+str(len(unexistent_files_in_path1)))
print("--------------------------------------------")

nb_empty_files_p1 = 0
nb_empty_files_p2 = 0
nb_empty_files_unexistent_in_path1 = 0
nb_empty_files_unexistent_in_path2 = 0

not_empty_p1 = []
not_empty_p2 = []
not_empty_files_unexistent_in_path2 = []
not_empty_files_unexistent_in_path1 = []

empty_files_p2 = []

for f1 in files_path1:
    with open(path1+f1) as f:
        lineList = f.readlines()
        if len(lineList) == 0:
            nb_empty_files_p1+=1
        else:
            not_empty_p1.append(f1)

for f2 in files_path2:
    with open(path2+f2) as f:
        lineList = f.readlines()
        if len(lineList) == 0:
            nb_empty_files_p2+=1
            empty_files_p2.append(f2)
        else:
            not_empty_p2.append(f2)

for u1 in unexistent_files_in_path1:
    with open(path2+u1) as f:
        lineList = f.readlines()
        if len(lineList) == 0:
            nb_empty_files_unexistent_in_path1+=1
        else:
            not_empty_files_unexistent_in_path1.append(u1)

for u2 in unexistent_files_in_path2:
    with open(path1+u2) as f:
        lineList = f.readlines()
        if len(lineList) == 0:
            nb_empty_files_unexistent_in_path2+=1
        else:
            not_empty_files_unexistent_in_path2.append(u2)

print("Nb of empty files in path1: "+str(nb_empty_files_p1))
print("Nb of empty files in path2: "+str(nb_empty_files_p2))
print("Nb of empty files in path1, unexistent in path2: "+str(nb_empty_files_unexistent_in_path2))
print("Nb of empty files in path2, unexistent in path1: "+str(nb_empty_files_unexistent_in_path1))

#print("not empty files from path2, unexisting in path1: ")
#for name in not_empty_files_unexistent_in_path1:
#    print(name)

#print("not empty files from path1, unexisting in path2: ")
#for name in not_empty_files_unexistent_in_path2:
#    print(name)

#print("empty files in path2")
#for f in empty_files_p2:
#    print(f)

if create_missing_files_in_p2:
    for unex in unexistent_files_in_path2:
        filename = path2 + unex
        f=open(filename, "a")
        f.close()




