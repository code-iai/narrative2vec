import sys
import time
from os import listdir, environ
from os.path import isdir, join

from narrative2vec.narrative import Narrative
import narrative2vec.knowrob_communication.knowrob_talker as talker

if __name__ == "__main__":
    args = sys.argv[1:]
    if len(args) == 2:
        neem_home_path = args[0]
        result_dir_path = args[1]

        if isdir(neem_home_path):
            talker.init_knowrob_talker()
            neem_names = listdir(neem_home_path)

            '''
            NEEM_HOME is intended to be set for a docker environment.
            KNOWROB outside the container cannot use 
            '''
            if environ.get('NEEM_HOME'):
                neem_home_path = environ.get('NEEM_HOME')

            if len(neem_names) == 0:
                print 'The neem source folder is empty'

            for neem_name in neem_names:
                neem_path = join(neem_home_path, neem_name)

                narrative = Narrative(neem_path)
                narrative.load()
                print 'Processing NEEM {}'.format(neem_path)
                narrative.transform_to_csv_file(result_dir_path)
                print 'DONE Processing NEEM {}'.format(neem_path)
            talker.close()
        else:
            print 'No vaild directory given: {}'.format(neem_home_path)
    else:
        print 'Please run the script with narrative2vec.py <path_to_neems> <path_to_the_csv_files>'
