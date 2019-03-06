import csv
from os import makedirs

import rdflib

from neem.logging_instance.action import Action
from neem.logging_instance.pose import Pose
from neem.logging_instance.reasoning_task import ReasoningTask
from ontology.neemNarrativeDefinitions import PERFORMED_IN_PROJECTION, PREDICATE, QUATERNION
from ontology.ontologyHandler import get_uri
from os.path import join, basename, exists


def get_vector_definition():
    definition = ['id',
                  'type',
                  'startTime',
                  'endTime',
                  'duration',
                  'success',
                  'failure',
                  'parent',
                  'next',
                  'previous',
                  'object_acted_on',
                  'object_type',
                  'bodyPartsUsed',
                  'arm',
                  'grasp']

    return definition


def get_reasoning_task_vector_definition():
    definition = ['id',
                  'action_id',
                  'startTime',
                  'endTime',
                  'duration',
                  'predicate',
                  'parameters',
                  'result']
    return definition


def get_poses_vector_definition():
    definition = ['id',
                  'reasoning_task_id',
                  't_x',
                  't_y',
                  't_z',
                  'q_x',
                  'q_y',
                  'q_z',
                  'q_w']
    return definition


class Narrative:
    def __init__(self, path_to_narrative_file):
        self._pathToNarrativeFile_ = path_to_narrative_file
        self._graph_ = self._init_graph()
        self.reasoning_tasks = None
        self.poses = None
        self.name = basename(path_to_narrative_file).split('.')[0]

    def get_reasoning_tasks(self):
        if self.reasoning_tasks is None:

            self.reasoning_tasks = []
            reasoning_tasks = self._query_all_reasoning_tasks_()

            for reasoning_task in reasoning_tasks:
                self.reasoning_tasks.append(self._reasoning_task_2_vec_(reasoning_task))

        return self.reasoning_tasks

    def get_poses(self):
        if self.poses is None:

            self.poses = []
            poses = self._query_all_poses_()

            for pose in poses:
                self.poses.append(self._pose_2_vec_(pose))

        return self.poses

    def _pose_2_vec_(self, pose_uri):
        pose = Pose(pose_uri, self._graph_)

        vec = [pose.get_id()]
        vec.append(pose.get_reasoning_task__id())
        vec.extend(pose.get_translation())
        vec.extend(pose.get_quaternion())

        return vec

    def _reasoning_task_2_vec_(self, reasoning_task_uri):
        reasoning_task = ReasoningTask(reasoning_task_uri, self._graph_)

        vec = [reasoning_task.get_id()]
        vec.append(reasoning_task.get_action_id())
        vec.append(reasoning_task.get_start_time_())
        vec.append(reasoning_task.get_end_time())
        vec.append(reasoning_task.get_end_time() - reasoning_task.get_start_time_())
        vec.append(reasoning_task.get_predicate())
        vec.append(reasoning_task.get_parameters())
        vec.append(reasoning_task.get_result())

        return vec

    def toVecs(self):
        actions = self.get_all_actions()

        vecs = []

        for action in actions:
            vecs.append(self.toVec(action))

        return vecs

    def toVec(self, action_uri):
        action = Action(action_uri, self._graph_)

        vector = [action.get_id()]
        vector.append(action.get_type())
        vector.append(action.get_start_time_())
        vector.append(action.get_end_time())
        vector.append(action.get_end_time() - action.get_start_time_())
        vector.append(action.is_successful())
        vector.append(action.get_failure())

        parent = action.get_parent_action()

        if parent:
            vector.append(parent.get_type())
        else:
            vector.append('')

        next_action = action.get_next_action()
        if next_action:
            vector.append(next_action.get_type())
        else:
            vector.append('')

        previous_action = action.get_previous_action()
        if previous_action:
            vector.append(previous_action.get_type())
        else:
            vector.append('')

        vector.append(action.get_object_acted_on())
        vector.append(action.get_object_type())
        vector.append(action.get_body_parts_used())
        vector.append(action.get_arm())
        vector.append(action.get_grasp())

        return vector

    def _query_all_reasoning_tasks_(self):
        return self._graph_.subjects(predicate=get_uri(PREDICATE))

    def _query_all_poses_(self):
        return self._graph_.subjects(predicate=get_uri(QUATERNION))

    def get_all_actions(self):
        return self._graph_.subjects(predicate=get_uri(PERFORMED_IN_PROJECTION))

    def get_all_action_types(self):
        action_types = set()
        actions = self.get_all_actions()

        for action in actions:
            action_type = self._get_action_type_of_action(action)
            action_types.add(str(action_type))

        return action_types

    def transform_to_csv_file(self, path_destination_dir):
        path_to_csv = join(path_destination_dir, self.name)

        if not exists(path_to_csv):
            makedirs(path_to_csv)
            
        self._write_narrative_vectors_to_csv_file_(path_to_csv)
        self._write_reasoning_tasks_to_csv_file_(path_to_csv)
        self._write_poses_to_csv_file_(path_to_csv)

    def _write_narrative_vectors_to_csv_file_(self, result_dir_path):
        vecs = self.toVecs()
        narrative_path = join(result_dir_path, 'narrative.csv')

        with open(narrative_path, 'wb') as csvfile:
            vec_writer = csv.writer(csvfile, delimiter=';')
            vec_writer.writerow(get_vector_definition())

            for vec in vecs:
                vec_writer.writerow(vec)

    def _write_reasoning_tasks_to_csv_file_(self, result_dir_path):
        vecs = self.get_reasoning_tasks()
        narrative_path = join(result_dir_path, 'reasoning_tasks.csv')

        with open(narrative_path, 'wb') as csvfile:
            vec_writer = csv.writer(csvfile, delimiter=';')
            vec_writer.writerow(get_vector_definition())

            for vec in vecs:
                vec_writer.writerow(vec)

    def _write_poses_to_csv_file_(self, result_dir_path):
        vecs = self.get_poses()
        narrative_path = join(result_dir_path, 'poses.csv')

        with open(narrative_path, 'wb') as csvfile:
            vec_writer = csv.writer(csvfile, delimiter=';')
            vec_writer.writerow(get_vector_definition())

            for vec in vecs:
                vec_writer.writerow(vec)

    def _init_graph(self):
        graph = rdflib.Graph()
        graph.load(self._pathToNarrativeFile_)
        return graph

