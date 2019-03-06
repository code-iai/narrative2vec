from neem.logging_instance.logging_instance import LoggingInstance
from neem.ontology.neemNarrativeDefinitions import \
    TASK_SUCCESS, PREVIOUS_ACTION, NEXT_ACTION, SUB_ACTION, \
    OBJECT_ACTED_ON, BODY_PARTS_USED, OBJECT_TYPE, GRASP, FAILURE, ARM

from neem.ontology.ontologyHandler import get_suffix_of_uri


class Action(LoggingInstance):
    def get_parent_action(self):
        parent_uri = self._get_property_(SUB_ACTION)
        return self._turn_uri_into_action(parent_uri)

    def get_next_action(self):
        return self._get_sibling_action_(NEXT_ACTION)

    def get_previous_action(self):
        return self._get_sibling_action_(PREVIOUS_ACTION)

    def get_object_acted_on(self):
        object_type = self._get_property_(OBJECT_ACTED_ON)

        if object_type:
            object_type = object_type.split('/')[-1].split("-")[0]

        return object_type

    def get_object_type(self):
        object_type = self._get_property_(OBJECT_TYPE)

        if object_type:
            object_type = object_type.split('/')[-1].split("-")[0]

        return object_type

    def get_grasp(self):
        grasp = self._get_property_(GRASP)

        if grasp:
            grasp = grasp.split(':')[1]

        return grasp

    def get_body_parts_used(self):
        body_parts_used = self._get_property_(BODY_PARTS_USED)
        if body_parts_used:
            return get_suffix_of_uri(body_parts_used)

        return body_parts_used

    def get_arm(self):
        arm = self._get_property_(ARM)
        if arm:
            return get_suffix_of_uri(arm)

        return arm

    def get_failure(self):
        failure = self._get_property_(FAILURE)

        return failure

    def _get_sibling_action_(self, sibling_type):
        sibling_uri = self._get_property_(sibling_type)
        return self._turn_uri_into_action(sibling_uri)

    def _turn_uri_into_action(self, action_uri):
        if action_uri:
            return Action(action_uri, self._graph_)
        return None

    def is_successful(self):
        value = self._get_property_(TASK_SUCCESS)

        return str(value) == 'true'

    def get_type(self):
        # http://knowrob.org/kb/knowrob.owl#PuttingDownAnObject_HLOUBZDW
        return get_suffix_of_uri(self.uri).split('_')[0]
