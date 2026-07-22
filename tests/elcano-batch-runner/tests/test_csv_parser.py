import os
import unittest
import csv

from elcano_batch_runner.csv_parser import parse_csv

class TestCSVParser(unittest.TestCase):

    def setUp(self):
        self.test_file_path = os.path.join(os.path.dirname(__file__), 'test_data.csv')
        with open(self.test_file_path, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['time_ms', 'CANID', 'nbytes', 'speed_cmPs', 'brake', 'mode', 'angle_tenths'])
            writer.writerow([0, 350, 6, 1500, 0, 1, 0])
            writer.writerow([1500, 'sim_speed_cmPs', 108, 216, 378, 432])
            writer.writerow([1500, 350, 6, 0, 2, 1, 250])
            writer.writerow([3200, 'sim_speed_cmPs', 0, 0, 0, 20])
            writer.writerow([4200, 'actual_angle_tenths', 220, 240, 250, 260])

    def tearDown(self):
        os.remove(self.test_file_path)

    def test_parse_csv(self):
        expected_output = [
            {'time_ms': 0, 'CANID': 350, 'nbytes': 6, 'speed_cmPs': 1500, 'brake': 0, 'mode': 1, 'angle_tenths': 0},
            {'time_ms': 1500, 'CANID': 'sim_speed_cmPs', 'nbytes': 108, 'speed_cmPs': 216, 'brake': 378, 'mode': 432},
            {'time_ms': 1500, 'CANID': 350, 'nbytes': 6, 'speed_cmPs': 0, 'brake': 2, 'mode': 1, 'angle_tenths': 250},
            {'time_ms': 3200, 'CANID': 'sim_speed_cmPs', 'nbytes': 0, 'speed_cmPs': 0, 'brake': 0, 'mode': 20},
            {'time_ms': 4200, 'CANID': 'actual_angle_tenths', 'nbytes': 220, 'speed_cmPs': 240, 'brake': 250, 'mode': 260}
        ]
        
        parsed_data = parse_csv(self.test_file_path)
        self.assertEqual(parsed_data, expected_output)

if __name__ == '__main__':
    unittest.main()