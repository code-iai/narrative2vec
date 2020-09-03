from narrative2vec.ontology.ontologyHandler import get_suffix_of_uri, get_ease_uri, get_dul_uri


def _get_time_from_timepoint(timepoint):
    return float(timepoint)


def _get_first_rdf_query_result(result):
    try:
        return list(result)[0]
    except IndexError:
        return None


class LoggingInstance(object):
    def __init__(self, context, time_interval, graph):
        self.context = context
        self.time_interval = time_interval
        self._graph_ = graph

    def get_id(self):
        return get_suffix_of_uri(self.context.action_uri)

    def get_start_time_(self):
        return self.time_interval.start_time

    def get_end_time(self):
        return self.time_interval.end_time

    def _get_action_property_(self, uri):
        return self._get_property_(self.context.action_uri, uri)

    def get_type_property(self, uri):
        return self._get_property_(self.context.type_uri, uri)

    def _get_property_(self, context_uri, uri):
        action_property = self._graph_.objects(context_uri, uri)
        if len(action_property) == 0:
            return None
        else:
            return action_property[0]