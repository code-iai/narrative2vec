from os.path import join
from neem.narrative import Narrative

NEEM_TEST_FILE_PATH = join('test_files', '1551794276507338.owl')


def test_should_read_neem_narrative_file():
    Narrative(NEEM_TEST_FILE_PATH)
    assert True