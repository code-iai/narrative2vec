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
        self.failure = ''
        self.parent_uri = ''
        self.grasp_uri = ''
        self.arm = ''
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
        if self.parent_uri:
            return get_suffix_of_uri(self.parent_uri)
        return ''

    def get_next_action(self):
        return self._get_sibling_action_(NEXT_ACTION)

    def get_previous_action(self):
        return self._get_sibling_action_(PREVIOUS_ACTION)

    def get_object_acted_on(self):
        query =  "ask(triple(dul:'{}',dul:'hasParticipant',Object))".format(self.get_id())
        solution = self._graph_.send_query(query)
        object_acted_on = None
        if solution:
            object_acted_on = get_suffix_of_uri(solution.get('Object'))
        elif self.get_type() == 'Accessing' or self.get_type() == 'Sealing':
            results = self._graph_.send_query("findall(O,ask(triple(dul:'{}',rdfs:'comment', O)),R).".format(self.get_id()))
            for result in results.get('R'):
                comment = result
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
        if self.grasp_uri:
            #http://www.ease-crc.org/ont/EASE.owl#FrontGrasp_TUYVDNPS
            return get_suffix_of_uri(self.grasp_uri).split('Grasp_')[0]

        return ''

    def get_body_parts_used(self):
        body_parts_used = self._get_property_(BODY_PARTS_USED)

        if body_parts_used:
            return get_suffix_of_uri(body_parts_used)
        elif self._equated_action:
            body_parts_used = self._equated_action.get_body_parts_used()

        return body_parts_used

    def get_arm(self):
        if self.get_type() == 'Grasping' or self.get_type() == 'Placing':
            results = self._graph_.send_query("findall(O,ask(triple(dul:'{}',rdfs:'comment', O)),R).".format(self.get_id()))
            for result in results.get('R'):
                if result.startswith("Unknown Parameter: :GRIPPER -####-"):
                    return result.split("Unknown Parameter: :GRIPPER -####-")[1]
                elif result.startswith("Unknown Parameter: :ARM -####-"):
                    return result.split("Unknown Parameter: :ARM -####-")[1]

        return ''

    def get_failure(self):
        return get_suffix_of_uri(self.failure)


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
        return self.failure is ''

    def get_type(self):
        # http://knowrob.org/kb/knowrob.owl#PuttingDownAnObject_HLOUBZDW
        return get_suffix_of_uri(self.context.type_uri).split('_')[0]