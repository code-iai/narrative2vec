import sys

from os import listdir
from os.path import isdir, join
from neem.narrative import Narrative

if __name__ == "__main__":
    args = sys.argv[1:]
    path = args[0]
    result_dir_path = args[1]

    if isdir(path):
        for neemName in listdir(path):
            neem_path = join(path, neemName)

            narrative = Narrative(neem_path)
            narrative.transform_to_csv_file(result_dir_path)
    else:
        narrative = Narrative(path)
        narrative.transform_to_csv_file(result_dir_path)