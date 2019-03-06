import csv
import sys
from os import listdir, makedirs
from os.path import isdir, join, exists, basename
from neem.narrative import Narrative, get_vector_definition, get_reasoning_task_vector_definition, \
    get_poses_vector_definition
import shutil


def write_reasoning_task_to_csv(vecs, result_dir_path):
    narrative_path = join(result_dir_path, 'reasoning_tasks.csv')

    with open(narrative_path, 'wb') as csvfile:
        vec_writer = csv.writer(csvfile, delimiter=';')
        vec_writer.writerow(get_reasoning_task_vector_definition())

        for vec in vecs:
            vec_writer.writerow(vec)


def write_narrative_vector_to_csv(vecs, result_dir_path):
    narrative_path = join(result_dir_path, 'narrative.csv')

    with open(narrative_path, 'wb') as csvfile:
        vec_writer = csv.writer(csvfile, delimiter=';')
        vec_writer.writerow(get_vector_definition())

        for vec in vecs:
            vec_writer.writerow(vec)


def write_poses_to_csv(vecs, result_dir_path):
    narrative_path = join(result_dir_path, 'poses.csv')

    with open(narrative_path, 'wb') as csvfile:
        vec_writer = csv.writer(csvfile, delimiter=';')
        vec_writer.writerow(get_poses_vector_definition())

        for vec in vecs:
            vec_writer.writerow(vec)


def transform_neem_to_csv(neem_path, result_dir_path):
    experiment_name = basename(neem_path).split('.')[0]
    narrative = Narrative(neem_path)

    store_path = join(result_dir_path, experiment_name)

    if exists(store_path):
        shutil.rmtree(store_path)

    makedirs(store_path)

    vecs = narrative.toVecs()

    write_narrative_vector_to_csv(vecs, store_path)
    write_reasoning_task_to_csv(narrative.get_reasoning_tasks(), store_path)
    write_poses_to_csv(narrative.get_poses(), store_path)


if __name__ == "__main__":
    args = sys.argv[1:]
    path = args[0]
    result_dir_path = args[1]

    if isdir(path):
        for neemName in listdir(path):
            neem_path = join(path, neemName)
            transform_neem_to_csv(neem_path, result_dir_path)
    else:
        transform_neem_to_csv(path, result_dir_path)