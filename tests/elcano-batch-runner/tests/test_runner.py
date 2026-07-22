import os
import glob
import csv

def run_tests():
    test_directory = r'C:\projects\elcano-Bridge\tests'
    csv_files = glob.glob(os.path.join(test_directory, '*.csv'))

    for csv_file in csv_files:
        print(f'Running tests for: {csv_file}')
        with open(csv_file, mode='r') as file:
            reader = csv.reader(file)
            for row in reader:
                # Here you would implement the logic to execute the test based on the CSV data
                print(f'Executing test with parameters: {row}')

if __name__ == '__main__':
    run_tests()