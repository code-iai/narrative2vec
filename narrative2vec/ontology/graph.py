import narrative2vec.knowrob_communication.knowrob_talker as talker


def init():
    talker.init_knowrob_talker()


def load(path_to_narrative_file):
    query = "remember('{0}'), " \
            "use_module('/home/koralewski/catkin_ws/ros_cram/src/knowrob/src/comm/ros/tf/tf_plugin.pl')," \
            "tf_mng_remember('{0}').".format(path_to_narrative_file)
    talker.get_all_solutions(query)


def subjects_objects(predicate_uri):
    query = "ask(triple(S,'{}',O)).".format(predicate_uri)
    solutions = talker.get_all_solutions(query)
    return [(solution.get('S'), solution.get('O')) for solution in solutions]


def subjects(predicate_uri, object_uri):
    query = "ask(triple(S,'{}','{}')).".format(predicate_uri, object_uri)
    solutions = talker.get_all_solutions(query)
    return [(solution.get('S')) for solution in solutions]


def objects(subject_uri, predicate_uri):
    query = "ask(triple('{}','{}',O)).".format(subject_uri, predicate_uri)
    solutions = talker.get_all_solutions(query)
    return [(solution.get('O')) for solution in solutions]


def is_concept_type_of(concept_uri, type_uri):
    query = "ask(instance_of('{}','{}')).".format(concept_uri, type_uri)
    solutions = talker.get_all_solutions(query)
    #solutions contains empty list if the query is false.If the query is true solutions contains an empty dict
    if len(solutions) == 1 and isinstance(solutions[0], dict):
        return True
    else:
        return False


def send_query(query):
    return talker.get_all_solutions(query)


def close():
    talker.close()