import os
import glob
import pandas as pd
import unittest

class TestScorer(unittest.TestCase):
    def setUp(self):
        self.test_files = glob.glob(os.path.join('C:\\projects\\elcano-Bridge\\tests', '*.csv'))

    def test_csv_files(self):
        for file in self.test_files:
            with self.subTest(file=file):
                df = pd.read_csv(file)
                self.assertFalse(df.empty, f'Test file {file} is empty.')
                # Add more assertions based on the expected structure of the CSV files

if __name__ == '__main__':
    unittest.main()