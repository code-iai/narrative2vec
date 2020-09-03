from narrative2vec.logging_instance.logging_instance import LoggingInstance, _get_first_rdf_query_result
from narrative2vec.ontology.neemNarrativeDefinitions import \
    TASK_SUCCESS, PREVIOUS_ACTION, NEXT_ACTION, SUB_ACTION, \
    OBJECT_ACTED_ON, BODY_PARTS_USED, OBJECT_TYPE, GRASP, FAILURE, ARM, EQUATE, EFFORT, SATISFIES, HAS_PARAMETER, \
    INCLUDES_CONCEPT, IS_SETTING_FOR, GRASPING_ORIENTATION_REGION, GRASPING_ORIENTATION, DESIGNED_ARTIFACT, \
    INCLUDES_OBJECT, HAS_CONSTITUENT
import re

from narrative2vec.ontology.ontologyHandler import get_suffix_of_uri, get_knowrob_uri, get_dul_uri, get_ease_uri


class Action(LoggingInstance):
    def __init__(self, context, time_interval, graph):
        super(Action, self).__init__(context, time_interval, graph)
        #self._object_map = self._init_object_map()

    def _init_object_map(self):
        result = {}
        query = "ask(instance_of(R,'{}')).".format(get_dul_uri(DESIGNED_ARTIFACT))
        solutions = self._graph_.send_query(query)

        for solution in solutions:
            object_uri = solution.get('R')
            object_id = get_suffix_of_uri(object_uri)
            object_type = self._get_object_type(object_id)
            result[object_uri] = object_type

        return result

    def _get_object_type(self, object_id):
        object_type = object_id.split('_')[0]

        if object_type == DESIGNED_ARTIFACT:
            query = "ask(triple('{}', rdfs:'comment', R)).".format(get_dul_uri(object_id))
            solutions = self._graph_.send_query(query)

            if len(solutions) == 1:
                solution = solutions[0].get('R')
                object_type = solution.split(':')[1]

        return object_type

    def get_parent_action(self):
        action_property = self._graph_.subjects(get_dul_uri(HAS_CONSTITUENT), self.context.action_uri)
        if action_property:
            return get_suffix_of_uri(action_property[0])

        return ''

    def get_next_action(self):
        return self._get_sibling_action_(NEXT_ACTION)

    def get_previous_action(self):
        return self._get_sibling_action_(PREVIOUS_ACTION)

    def get_object_acted_on(self):
        object_acted_on = self._get_action_property_(get_dul_uri(INCLUDES_OBJECT))
        if object_acted_on is not None:
            object_acted_on = get_suffix_of_uri(object_acted_on)
        elif self.get_type() == 'Accessing' or self.get_type() == 'Sealing':
            results = self._graph_.send_query("ask(triple(dul:'{}',rdfs:'comment', O)).".format(self.get_id()))
            for result in results:
                comment = result.get('O')
                search_result = re.search("\(URDF-NAME(.+)\)",comment)
                if search_result:
                    object_acted_on = search_result.groups()[0]
                    break
        return object_acted_on

    def get_object_type(self):
        object_type = self._get_action_property_(get_dul_uri(INCLUDES_OBJECT))
        if object_type is not None:
            object_type = self._object_map.get(object_type)

        return object_type

    def get_grasp(self):
        grasp = self.get_type_property(get_dul_uri(HAS_PARAMETER))

        if grasp and self._graph_.is_concept_type_of(grasp, get_ease_uri(GRASPING_ORIENTATION)):
            query = "ask(triple('{}','{}', O))."\
                    .format(grasp,
                            get_dul_uri(INCLUDES_CONCEPT))

            solutions = self._graph_.send_query(query)
            if len(solutions) == 1:
                grasp = solutions[0].get('O')
                if grasp:
                    #'http://www.ease-crc.org/ont/EASE.owl#FrontGrasp_TUYVDNPS'
                    grasp = get_suffix_of_uri(grasp).split('Grasp_')[0]

        return grasp

    def get_body_parts_used(self):
        body_parts_used = self._get_property_(BODY_PARTS_USED)

        if body_parts_used:
            return get_suffix_of_uri(body_parts_used)
        elif self._equated_action:
            body_parts_used = self._equated_action.get_body_parts_used()

        return body_parts_used

    def get_arm(self):
        arm = self._get_property_(ARM)
        if arm:
            return get_suffix_of_uri(arm)
        elif self._equated_action:
            return self._equated_action.get_arm()

    def get_failure(self):
        failure = self._get_action_property_(get_dul_uri(SATISFIES))

        if failure:
            return get_suffix_of_uri(str(failure))
        else:
            return ''

    def get_effort(self):
        effort = self._get_property_(EFFORT)

        if not effort and self._equated_action:
            return self._equated_action.get_effort()

        return effort

    def _get_sibling_action_(self, sibling_type):
        sibling_uri = self._get_property_(sibling_type)
        return self._turn_uri_into_action(sibling_uri)

    def _turn_uri_into_action(self, action_uri):
        if action_uri:
            return Action(action_uri, self._graph_)
        return None

    def is_successful(self):
        value = self._get_action_property_(get_dul_uri(SATISFIES))

        return value is None

    def get_type(self):
        # http://knowrob.org/kb/knowrob.owl#PuttingDownAnObject_HLOUBZDW
        return get_suffix_of_uri(self.context.type_uri).split('_')[0]