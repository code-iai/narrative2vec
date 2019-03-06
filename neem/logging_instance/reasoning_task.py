from neem.logging_instance.action import Action
from neem.logging_instance.logging_instance import LoggingInstance, _get_first_rdf_query_result
from neem.ontology.neemNarrativeDefinitions import PREDICATE
from neem.ontology.ontologyHandler import get_uri


class ReasoningTask(LoggingInstance):
    def get_parameters(self):
        return self._get_property_('parameters')

    def get_predicate(self):
        return self._get_property_(PREDICATE)

    def get_result(self):
        return self._get_property_('result')

    def get_action_id(self):
        action_property = self._graph_.subjects(get_uri('reasoningTask'), self.uri)
        action = _get_first_rdf_query_result(action_property)

        if action and not action.startswith('file://'):
            return Action(action, self._graph_).get_id()

        return ''