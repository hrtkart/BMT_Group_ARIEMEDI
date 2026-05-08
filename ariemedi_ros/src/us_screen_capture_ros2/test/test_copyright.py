from ament_copyright.main import main


def test_copyright():
    rc = main(argv=['.', 'test'])
    assert rc == 0
