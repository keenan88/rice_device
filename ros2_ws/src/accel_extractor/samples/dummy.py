import csv

samples_folder = '/home/keenan/Documents/rice_device/ros2_ws/src/accel_extractor/samples/'
file_path = samples_folder + 'samples1.csv'

samples = [[1,2,3,4], [4,3,2,5,6,7]]

with open(file_path, 'w', newline='') as csvfile:
                spamwriter = csv.writer(csvfile, delimiter=',',
                                        quotechar='|', quoting=csv.QUOTE_MINIMAL)
                for sample in samples:
                    spamwriter.writerow(sample)
            