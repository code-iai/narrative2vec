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
        print 'Query:{}'.format(query)
        return _prolog.once(query)
    else:
        print 'Cannot send query because talker is not alive.'


def get_all_solutions(query_str):
    return _send_query(query_str)


def close():
    rospy.signal_shutdown('Closing Talker')
    print ('Closing Talker')


def _is_talker_alive():
    return not rospy.is_shutdown()


