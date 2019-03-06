from os.path import join, abspath, dirname
from neem.narrative import Narrative

ABSOULTE_PATH_TO_NARRATIVE_TEST_PARENT_DIR = dirname(abspath(__file__))
NEEM_TEST_FILE_PATH = join(ABSOULTE_PATH_TO_NARRATIVE_TEST_PARENT_DIR, 'test_files', '1551794276507338.owl')


def test_should_read_neem_narrative_file():
    Narrative(NEEM_TEST_FILE_PATH)
    assert True