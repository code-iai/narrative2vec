import sys

from os import listdir
from os.path import isdir, join
from narrative2vec.narrative import Narrative


import rospy
from rosprolog_client import PrologException, Prolog
import narrative2vec.knowrob_communication.knowrob_talker as talker

if __name__ == "__main__":
    talker.init_knowrob_talker()
    talker.get_all_solutions("rdf_has(Action, dul:'isExecutedIn', _).")
    talker.close()
    # prolog = Prolog()
    # rospy.init_node('narrative2vec', anonymous=True)
    # while not rospy.is_shutdown():
    #     with prolog.query() as query:
    #         for solution in query.solutions():
    #             print solution
    #     rospy.signal_shutdown("Done processing NEEM.")
        # args = sys.argv[1:]
    # path = args[0]
    # result_dir_path = args[1]
    #
    # if isdir(path):
    #     for neemName in listdir(path):
    #         neem_path = join(path, neemName)
    #
    #         narrative = Narrative(neem_path)
    #         narrative.transform_to_csv_file(result_dir_path)
    # else:
    #     narrative = Narrative(path)
    #     narrative.transform_to_csv_file(result_dir_path)