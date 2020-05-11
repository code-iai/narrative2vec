import narrative2vec.knowrob_communication.knowrob_talker as talker


def init():
    talker.init_knowrob_talker()


def load(path_to_narrative_file):
    query = "mem_import('{}').".format(path_to_narrative_file)
    talker.get_all_solutions(query)


def subjects_objects(predicate_uri):
    query = "rdf_has(S,'{}',O).".format(predicate_uri)
    solutions = talker.get_all_solutions(query)
    return [(solution.get('S'), solution.get('O')) for solution in solutions]


def subjects(predicate_uri, object_uri):
    query = "rdf_has(S,'{}','{}').".format(predicate_uri, object_uri)
    solutions = talker.get_all_solutions(query)
    return [(solution.get('S')) for solution in solutions]


def objects(subject_uri, predicate_uri):
    query = "rdf_has('{}','{}',O).".format(subject_uri, predicate_uri)
    solutions = talker.get_all_solutions(query)
    return [(solution.get('O')) for solution in solutions]


def close():
    talker.close()