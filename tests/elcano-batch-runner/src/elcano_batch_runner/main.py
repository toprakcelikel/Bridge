import os
import glob
import subprocess

def run_csv_tests(test_directory):
    # Find all CSV files in the specified directory
    csv_files = glob.glob(os.path.join(test_directory, '*.csv'))
    
    # Execute each CSV test file
    for csv_file in csv_files:
        print(f'Running test for: {csv_file}')
        # Here you would call the function that processes the CSV file
        # For example, if you have a function called `process_csv` in your test_runner module:
        # subprocess.run(['python', 'src/elcano_batch_runner/test_runner.py', csv_file])
        # For now, we will just simulate the call
        print(f'Simulated execution of: {csv_file}')

if __name__ == '__main__':
    test_directory = r'C:\projects\elcano-Bridge\tests'
    run_csv_tests(test_directory)