# Elcano Batch Runner

## Overview
The Elcano Batch Runner is a Python application designed to execute a series of CSV test files located in a specified directory. It parses the CSV files, runs the tests defined within them, and generates reports based on the results. This project aims to facilitate automated testing for various scenarios, ensuring that the system behaves as expected under different conditions.

## Project Structure
```
elcano-batch-runner
├── src
│   ├── elcano_batch_runner
│   │   ├── __init__.py
│   │   ├── main.py
│   │   ├── csv_parser.py
│   │   ├── test_runner.py
│   │   ├── scorer.py
│   │   └── report.py
├── tests
│   ├── __init__.py
│   ├── test_csv_parser.py
│   ├── test_scorer.py
│   └── test_runner.py
├── config
│   └── settings.toml
├── requirements.txt
├── pyproject.toml
└── README.md
```

## Installation
To set up the Elcano Batch Runner, follow these steps:

1. Clone the repository:
   ```
   git clone <repository-url>
   cd elcano-batch-runner
   ```

2. Install the required dependencies:
   ```
   pip install -r requirements.txt
   ```

## Usage
To run the batch tests, execute the following command:
```
python -m elcano_batch_runner.main
```

Ensure that your CSV test files are located in the specified directory as defined in the configuration settings.

## Testing
Unit tests are provided to ensure the functionality of the application. To run the tests, use:
```
pytest
```

## Contributing
Contributions are welcome! Please submit a pull request or open an issue for any enhancements or bug fixes.

## License
This project is licensed under the MIT License. See the LICENSE file for more details.