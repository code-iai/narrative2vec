import sys
from os import listdir
from os.path import isdir, join

from narrative2vec.narrative import Narrative
import narrative2vec.knowrob_communication.knowrob_talker as talker

if __name__ == "__main__":
    args = sys.argv[1:]
    if len(args) == 2:
        path = args[0]
        result_dir_path = args[1]

        if isdir(path):
            talker.init_knowrob_talker()

            for neem_name in listdir(path):
                neem_path = join(path, neem_name)

                narrative = Narrative(neem_path)
                narrative.load()
                print 'Processing NEEM {}'.format(neem_path)
                narrative.transform_to_csv_file(result_dir_path)
                print 'DONE Processing NEEM {}'.format(neem_path)
            talker.close()
    else:
        print 'Please run the script with narrative2vec.py <path_to_neems> <path_to_the_csv_files>'
