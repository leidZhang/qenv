# 3rd party imports
import pytest
# custom imports
from qenv.utils.qlab_utlis import *

def test_get_random_deviate() -> None:
    boundary: float = 0.1
    for _ in range(10):
        res: float = get_random_deviate(boundary)
        assert -boundary <= res <= boundary, "Result out of expected range!"

def test_cal_waypoint_angles() -> None:
    # test array
    waypoints: np.ndarray = np.array([
        [0, 0], [-1, 0], [-2, -1],
        [-2, -2], [-1, -3], [0, -3],
        [1, -2], [1, -1], [0, 0]
    ])

    # expectation array
    expect: list = [
        math.pi, math.pi * 5 / 4, math.pi * 3 / 2,
        math.pi * 7 / 4, 0, math.pi / 4,
        math.pi / 2, math.pi * 3 / 4
    ]

    for i in range(len(waypoints) - 1):
        res: float = cal_waypoint_angles(waypoints, i)
        if res >= 0:
            assert res == pytest.approx(expect[i]), f"Expect: {expect[i]} at index {i}, but got {res}"
        else:
            assert res == pytest.approx(expect[i] - 2 * math.pi), f"Expect: {expect[i] - 2 * math.pi} at index {i}, but got {res}"

def test_get_waypoint_angles() -> None:
    # test empty array
    empty_sequence: np.ndarray = np.array([])
    result: list = get_waypoint_angles(empty_sequence)
    assert result == [], f"Expect: [], but got {result}"

    # test array with only one waypoint
    single_waypoint: np.ndarray = np.array([[0, 0]])
    result = get_waypoint_angles(single_waypoint)
    assert result == [], f"Expect: [], but got {result}"

    # test array with multiple waypoints
    waypoints: np.ndarray = np.array([[0, 0], [1, 1], [2, 2]])
    res: list = get_waypoint_angles(waypoints)
    expect: list = [math.pi / 4, math.pi / 4]
    for i in range(len(expect)):
        assert res[i] == pytest.approx(expect[i]), f"Expect {expect[i]} at index {i}, but got {res[i]}"

def test_get_deviate_state() -> None:
    original_position: list = [0, 0, 0]
    result: list = get_deviate_state(original_position)

    # test type and length
    assert isinstance(result, list), "The result should be an array"
    assert len(result) == 3, "The length of the array should be 3"
    # test range of the result
    for _ in range(100):
        result = get_deviate_state(original_position)
        x_deviation = result[0] - original_position[0]
        y_deviation = result[1] - original_position[1]
        orientation_deviation = result[2] - original_position[2]

        assert -0.1 <= x_deviation <= 0.1, "Deviation on x position out of expected range!"
        assert -0.1 <= y_deviation <= 0.1, "Deviation on x position out of expected range!"
        assert -0.15 <= orientation_deviation <= 0.15, "Deviation on orientation out of expected range!"