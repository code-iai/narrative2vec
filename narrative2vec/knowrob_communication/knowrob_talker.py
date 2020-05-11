import itertools

import rospy
from rosprolog_client import PrologException, Prolog

_prolog = None


def init_knowrob_talker():
    global _prolog
    _prolog = Prolog()
    rospy.init_node('narrative2vec', anonymous=True)


def _send_query(query):
    if _is_talker_alive():
        return _prolog.query(query)


def get_all_solutions(query_str):
    with _send_query(query_str) as query:
        return [x for x in query.solutions()]


def close():
    rospy.signal_shutdown('Closing Talker')
    print 'Closing Talker'


def _is_talker_alive():
    return not rospy.is_shutdown()


