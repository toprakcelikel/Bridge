from typing import List, Dict

def generate_report(test_results: List[Dict[str, str]]) -> str:
    report_lines = []
    report_lines.append("Test Results Report")
    report_lines.append("=" * 30)
    
    for result in test_results:
        report_lines.append(f"Test File: {result['test_file']}")
        report_lines.append(f"Status: {'PASS' if result['status'] else 'FAIL'}")
        report_lines.append(f"Score: {result['score']}")
        report_lines.append("-" * 30)
    
    return "\n".join(report_lines)

def save_report(report: str, output_file: str) -> None:
    with open(output_file, 'w') as file:
        file.write(report)

def main():
    # Example usage
    test_results = [
        {"test_file": "test1.csv", "status": True, "score": 85},
        {"test_file": "test2.csv", "status": False, "score": 55},
    ]
    
    report = generate_report(test_results)
    save_report(report, "test_report.txt")

if __name__ == "__main__":
    main()