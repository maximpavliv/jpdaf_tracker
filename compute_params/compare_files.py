import os

path1 = "./outputs/per_image/"
path2 = "./labels/treated"

files_path1 = os.listdir(path1)
files_path2 = os.listdir(path2)

unexistent_files_path1 = []
unexistent_files_path2 = []

for file_path1 in files_path1:
    if not os.path.isfile(path2+file_path1):
        unexistent_files_path2.append(file_path1)

for file_path2 in files_path2:
    if not os.path.isfile(path1+file_path2):
        unexistent_files_path1.append(file_path2)

print("Unexistent files in path 1: ")
for unex_file in unexistent_files_path1:
    print(unex_file)
print("--------------------------------------------")
print("Unexistent files in path 2: ")
for unex_file in unexistent_files_path2:
    print(unex_file)

