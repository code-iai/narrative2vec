from os import makedirs
from os.path import join, basename, exists

from ontology import graph
from narrative2vec.logging_instance.action import Action
from narrative2vec.logging_instance.logging_context import LoggingContext
from narrative2vec.logging_instance.pose import Pose
from narrative2vec.logging_instance.reasoning_task import ReasoningTask

import narrative_csv
from constants import action_table_header, reasoning_task_table_header, poses
from ontology.neemNarrativeDefinitions import IS_EXECUTED_IN, PREDICATE, QUATERNION, INCLUDES_ACTION
from ontology.ontologyHandler import get_knowrob_uri, get_dul_uri


class Narrative:
    def __init__(self, path_to_narrative_file):
        self._pathToNarrativeFile_ = path_to_narrative_file
        self.actions = None
        self._graph_ = None
        self.reasoning_tasks = None
        self.is_open = False
        self.poses = None
        self.name = basename(path_to_narrative_file).split('.')[0]
        self._path_to_csv = None

    def load(self):
        self._graph_ = self._init_graph()
        self.is_open = True

    def get_reasoning_tasks(self):
        if self.reasoning_tasks is None:

            self.reasoning_tasks = []
            reasoning_tasks = self._query_all_reasoning_tasks_()

            for reasoning_task in reasoning_tasks:
                self.reasoning_tasks.append(self._reasoning_task_2_vec_(reasoning_task))

        return self.reasoning_tasks

    def get_poses(self):
        if self.poses is None:
            self.poses = self._query_all_poses_()

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
        if not self.actions:
            self.actions = self.get_all_actions()

        return [self.toVec(x) for x in self.actions]

    def toVec(self, action):
        action_start_time = action.get_start_time_()
        action_end_time = action.get_end_time()

        vector = [
            action.get_id(),
            action.get_type(),
            action_start_time,
            action_end_time,
            action_end_time - action_start_time,
            action.is_successful(),
            action.get_failure(),
            '',
            '',
            '',
            action.get_object_acted_on(),
            action.get_object_type(),
            '',
            '',
            action.get_grasp(),
            '',
        ]

        # vector = [
        #     action.get_id(),
        #     action.get_type(),
        #     action_start_time,
        #     action_end_time,
        #     action_end_time - action_start_time,
        #     action.is_successful(),
        #     action.get_failure(),
        #     '',
        #     '',
        #     '',
        #     action.get_object_acted_on(),
        #     action.get_object_type(),
        #     action.get_body_parts_used(),
        #     action.get_arm(),
        #     action.get_grasp(),
        #     action.get_effort()
        # ]

        # try:
        #     vector[7] = action.get_parent_action().get_id()
        # except AttributeError:
        #     pass
        # try:
        #     vector[8] = action.get_next_action().get_id()
        # except AttributeError:
        #     pass
        # try:
        #     vector[9] = action.get_previous_action().get_id()
        # except AttributeError:
        #     pass

        return vector

    def _query_all_reasoning_tasks_(self):
        return self._graph_.subjects(predicate=get_knowrob_uri(PREDICATE))

    def _query_all_poses_(self):
        grasping_actions = filter(lambda action: action.get_type() == 'Grasping' and action.get_object_acted_on(), self.actions)
        rows = []
        id = 0

        for grasping_action in grasping_actions:
            query = "ask([triple('{}',dul:'hasTimeInterval',_O), triple(_O, ease:'hasIntervalEnd', _T2)])," \
                "time_scope(=<(_T2), >=(_T2), _QScope), tf_get_pose('{}', " \
                "['base_footprint',Position,Orientation], _QScope, _),!.".format(grasping_action.context.action_uri, grasping_action.get_object_acted_on())
            solutions = self._graph_.send_query(query)
            rows.append([id, grasping_action.get_id(), solutions[0].get('Position'), solutions[0].get('Orientation')])
            id += 1

        return rows

    def get_all_actions(self):
        result = [x for x in self._graph_.subjects_objects(get_dul_uri(IS_EXECUTED_IN))]

        return [Action(context, self._graph_) for context in [LoggingContext(action, task) for action, task in result]]

    def get_all_action_types(self):
        action_types = set()
        actions = self.get_all_actions()

        for action in actions:
            action_type = self._get_action_type_of_action(action)
            action_types.add(str(action_type))

        return action_types

    def transform_to_csv_file(self, path_destination_dir):
        self._path_to_csv = join(path_destination_dir, self.name)

        if not exists(self._path_to_csv):
            makedirs(self._path_to_csv)

        self._write_actions_to_csv_file_()
        #self._write_reasoning_tasks_to_csv_file_()
        self._write_poses_to_csv_file_()


    def _write_actions_to_csv_file_(self):
        csv_file_path = join(self._path_to_csv, 'actions.csv')
        narrative_csv.write(action_table_header.get_definition(), self.toVecs(), csv_file_path)

    def _write_reasoning_tasks_to_csv_file_(self):
        csv_file_path = join(self._path_to_csv, 'reasoning_tasks.csv')
        narrative_csv.write(reasoning_task_table_header.get_definition(), self.get_reasoning_tasks(), csv_file_path)

    def _write_poses_to_csv_file_(self):
        csv_file_path = join(self._path_to_csv, 'poses.csv')
        narrative_csv.write(poses.get_definition(), self.get_poses(), csv_file_path)

    def _init_graph(self):
        graph.load(self._pathToNarrativeFile_)
        return graph

