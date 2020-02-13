import os
import numpy as np
from scipy.optimize import linear_sum_assignment
from math import sqrt
from statistics import mean

image_height = 540
image_width = 960

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

images_measurements = []

for filename in label_files:
    print("------------------------------------------------------------")
    print("frame: "+filename)
    #read lines in label
    lines_label_str = []
    label_f = open(label_path + filename, 'r')
    while True:
        line = label_f.readline() 
        if not line: 
            break
        split_line = line.split()
        lines_label_str.append(split_line)
    label_f.close() 

    #read lines in output
    lines_output_str = []
    output_f = open(output_path + filename, 'r')
    while True:
        line = output_f.readline() 
        if not line: 
            break
        split_line = line.split()
        lines_output_str.append(split_line)
    output_f.close() 

    #convert to int
    bboxes_label = []
    for line in lines_label_str:
        bbox = []
        for nb in line:
            bbox.append(int(nb))
            #nb = (int)(nb)
        bboxes_label.append(bbox)
    bboxes_output = []
    for line in lines_output_str:
        bbox = []
        for nb in line:
            bbox.append(int(nb))
            #nb = (int)(nb)
        bboxes_output.append(bbox)

    nb_objects=len(bboxes_label)

    #compute centers 
    label_centers = []
    for line in bboxes_label:
        center = [(line[0]+line[2])/2, (line[1]+line[3])/2]
        label_centers.append(center)
    output_centers = []
    for line in bboxes_output:
        center = [(line[0]+line[2])/2, (line[1]+line[3])/2]
        output_centers.append(center)

    
    #find corresponding labels and detections
    cost =np.zeros((len(label_centers), len(output_centers)))
    print("nb of label centers: "+str(len(label_centers))+", nb of output centers: "+str(len(output_centers)))
    for i in range(0, len(label_centers)):
        for j in range(0, len(output_centers)):
            cost[i, j] = sqrt((label_centers[i][0]-output_centers[j][0])**2 + (label_centers[i][1]-output_centers[j][1])**2)
    print("cost array:")
    print(cost)
    row_ind, col_ind = linear_sum_assignment(cost)
    print("solved label indexes")
    print(row_ind)
    print("solved corresponding detection indexes")
    print(col_ind)

    #first attributions of not detected object and false measurements, based on number of detections and labels
    nb_not_detected_object = len(label_centers) - (row_ind.shape)[0]
    nb_false_measurements = len(output_centers) - (col_ind.shape)[0]
    print("primary nb of not detected objects: "+str(nb_not_detected_object))
    print("primary nb of false measurements: "+str(nb_false_measurements))

    #checking if labels and corresponding measurements have a non-zero intersections area. If yes - it means that a undetected object and false measurement occured at the same time!
    #TODO We probably can imagine cases where, due to a simultaneous false detection and undetected object case, uncorrect correspondance between labels and detections has been established. Maybe something different than linear_sum_assignment needs to be used! These specific case are ignored for now.
    correct_detections = []
    for i in range(0, len(row_ind)):
        label_index = row_ind[i]
        ouput_index = col_ind[i]
        if(bboxes_label[label_index][0] >= bboxes_output[ouput_index][2] or
           bboxes_label[label_index][1] >= bboxes_output[ouput_index][3] or
           bboxes_label[label_index][2] <= bboxes_output[ouput_index][0] or
           bboxes_label[label_index][3] <= bboxes_output[ouput_index][1]):
            nb_not_detected_object += 1
            nb_false_measurements += 1
        else:
            correct_detections.append([label_centers[label_index], output_centers[ouput_index]])

    print("final nb of not detected objects: "+str(nb_not_detected_object))
    print("final nb of false measurements: "+str(nb_false_measurements))

    nb_correct_detections = len(correct_detections)
    print("nb of correct detection: "+str(nb_correct_detections))
    detection_noise = []
    for correct_detection in correct_detections:
        detection_noise.append([abs(float(correct_detection[0][0]) - float(correct_detection[1][0])), abs(float(correct_detection[0][1]) - float(correct_detection[1][1]))])
    
        
    
    image_measurements = {"nb_objects" : nb_objects, "nb_not_detected_object" : nb_not_detected_object, "nb_false_measurements" : nb_false_measurements, "nb_correct_detections" : nb_correct_detections, "detection_noise" : detection_noise}
    
    images_measurements.append(image_measurements)
        
print("---------------------------------------------------------")
print("---------------------------------------------------------")
print("THE COMPUTED PARAMETERS ARE THE FOLLOWING")
PD_per_image = []
for image_measurements in images_measurements:
    if image_measurements["nb_objects"] != 0:
        PD_per_image.append(image_measurements["nb_correct_detections"]/image_measurements["nb_objects"])

PD = mean(PD_per_image)
print("PD: "+str(PD))

nb_false_detections = []
for image_measurements in images_measurements:
    nb_false_detections.append(image_measurements["nb_false_measurements"])
mean_false_detection = mean(nb_false_detections)
print("mean of false detections: "+str(mean_false_detection))

false_measurements_density = mean_false_detection/(image_width*image_height)
print("false_measurements_density: "+str(false_measurements_density))
noise_per_detection = []
for image_measurements in images_measurements:
    if image_measurements["nb_correct_detections"] != 0:
        for noise_measurement in image_measurements["detection_noise"]:
            noise_per_detection.append(noise_measurement)

mean_noise_x = mean(noise_per_detection[:][0])
mean_noise_y = mean(noise_per_detection[:][1])

print("Mean noise in x: "+str(mean_noise_x)+", mean noise in y: "+str(mean_noise_y))












