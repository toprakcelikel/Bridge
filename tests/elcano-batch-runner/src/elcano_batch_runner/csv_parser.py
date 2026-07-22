def read_csv(file_path):
    import csv
    data = []
    with open(file_path, mode='r', newline='') as csvfile:
        csv_reader = csv.reader(csvfile)
        for row in csv_reader:
            data.append(row)
    return data

def parse_csv_data(csv_data):
    parsed_data = []
    for row in csv_data:
        if len(row) > 0:
            parsed_data.append({
                'time_ms': int(row[0]),
                'CANID': row[1],
                'nbytes': int(row[2]),
                'speed_cmPs': int(row[3]),
                'brake': int(row[4]),
                'mode': int(row[5]),
                'angle_tenths': int(row[6]) if len(row) > 6 else None
            })
    return parsed_data

def get_test_files(directory):
    import os
    return [os.path.join(directory, f) for f in os.listdir(directory) if f.endswith('.csv')]