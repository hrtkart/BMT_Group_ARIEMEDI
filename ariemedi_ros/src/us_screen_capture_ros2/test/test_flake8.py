from ament_flake8.main import main_with_errors


def test_flake8():
    rc, _ = main_with_errors(argv=[])
    assert rc == 0
