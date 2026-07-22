import os
import glob
import csv

def run_tests_in_directory(directory):
    csv_files = glob.glob(os.path.join(directory, '*.csv'))
    results = []

    for csv_file in csv_files:
        result = run_test(csv_file)
        results.append(result)

    return results

def run_test(csv_file):
    # Placeholder for test execution logic
    # Here you would implement the logic to read the CSV file and execute the tests
    with open(csv_file, 'r') as file:
        reader = csv.reader(file)
        for row in reader:
            print(f"Running test with data: {row}")
            # Implement test logic based on CSV data
    return f"Test executed for {csv_file}"

if __name__ == "__main__":
    test_directory = r"C:\projects\elcano-Bridge\tests"
    results = run_tests_in_directory(test_directory)
    for result in results:
        print(result)