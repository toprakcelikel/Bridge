def calculate_score(actual_angle, expected_angle):
    score = max(0, 100 - abs(actual_angle - expected_angle))
    return score

def evaluate_test_results(test_results):
    scores = []
    for result in test_results:
        actual_angle = result['actual_angle']
        expected_angle = result['expected_angle']
        score = calculate_score(actual_angle, expected_angle)
        scores.append(score)
    return scores

def determine_pass_fail(scores, passing_score=60):
    return [score >= passing_score for score in scores]