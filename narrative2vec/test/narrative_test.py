import shutil
from os.path import join, abspath, dirname, exists
from os import listdir, makedirs
from narrative2vec.narrative import Narrative
import narrative2vec.knowrob_communication.knowrob_talker as talker

ABSOULTE_PATH_TO_NARRATIVE_TEST_PARENT_DIR = dirname(abspath(__file__))
NEEM_TEST_FILE_PATH = join(ABSOULTE_PATH_TO_NARRATIVE_TEST_PARENT_DIR, '1599047476.7768126')
DESTINATION_PATH_DIR = join(ABSOULTE_PATH_TO_NARRATIVE_TEST_PARENT_DIR, 'test_result')


def test_should_convert_narrative_to_csv_file():
    if exists(DESTINATION_PATH_DIR):
        shutil.rmtree(DESTINATION_PATH_DIR)

    makedirs(DESTINATION_PATH_DIR)
    talker.init_knowrob_talker()
    narrative = Narrative(NEEM_TEST_FILE_PATH)
    print 'Loading ...'
    narrative.load()
    print 'Loading done'
    narrative.transform_to_csv_file(DESTINATION_PATH_DIR)
    talker.close()

    narrative_dir_path = join(DESTINATION_PATH_DIR, narrative.name)
    test_result_files = listdir(narrative_dir_path)
    assert len(test_result_files) == 2
