import narrative2vec.knowrob_communication.knowrob_talker as talker


def init():
    talker.init_knowrob_talker()


def load(path_to_narrative_file):

    #"tripledb_load('package://knowrob/owl/knowrob.owl', [graph(tbox), namespace(knowrob)]),"\
    query = "use_module(library('db/mongo/client')),tripledb_drop(), forall(mng_collection(roslog, Coll), mng_drop(roslog, Coll)),ensure_loaded('/home/koralewski/catkin_ws/ros_cram/src/knowrob/src/ros/tf/tf_plugin.pl')," \
            "remember('{0}').".format(path_to_narrative_file)
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


def send_query_all(query):
    return talker.get_all_solutions(query)

def send_query_once(query):
    return talker.get_first_solution(query)

def close():
    talker.close()