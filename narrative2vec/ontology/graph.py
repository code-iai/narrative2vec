import narrative2vec.knowrob_communication.knowrob_talker as talker


def init():
    talker.init_knowrob_talker()


def load(path_to_narrative_file):
    query = "mem_import('{}').".format(path_to_narrative_file)
    return talker.get_first_solution(query)


def close():
    talker.close()