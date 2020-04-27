from narrative2vec.ontology.neemNarrativeDefinitions import START_TIME, END_TIME, INCLUDES_TIME
from narrative2vec.ontology.ontologyHandler import get_suffix_of_uri, get_ease_uri, get_dul_uri


def _get_time_from_timepoint(timepoint):
    return float(timepoint)


def _get_first_rdf_query_result(result):
    try:
        return list(result)[0]
    except IndexError:
        return None


class LoggingInstance(object):
    def __init__(self, context, graph):
        self.context = context
        self._graph_ = graph

    def get_id(self):
        return get_suffix_of_uri(self.context.plan_uri)

    def get_start_time_(self):
        return self._get_time_(START_TIME)

    def get_end_time(self):
        return self._get_time_(END_TIME)

    def _get_time_(self, time_type):
        time_instance = self._get_plan_property_(get_dul_uri(INCLUDES_TIME))
        if time_instance:
            time = _get_first_rdf_query_result(self._graph_.objects(time_instance, get_ease_uri(time_type)))
            return _get_time_from_timepoint(time)

        return 0.0

    def _get_plan_property_(self, uri):
        return self._get_property_(self.context.plan_uri, uri)

    def _get_property_(self, context_uri, uri):
        action_property = self._graph_.objects(context_uri, uri)
        return _get_first_rdf_query_result(action_property)