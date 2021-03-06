from uriDefinitions import KNOWROB_URI, DUL_URI, EASE_URI

def get_ease_uri(suffix):
    return _build_uri_ref_with_suffix(EASE_URI, suffix)


def get_dul_uri(suffix):
    return _build_uri_ref_with_suffix(DUL_URI, suffix)


def get_knowrob_uri(suffix):
    return _build_uri_ref_with_suffix(KNOWROB_URI, suffix)


def _build_uri_ref_with_suffix(uri, suffix):
    return uri + "#" + suffix


def get_suffix_of_uri(uri):
    #http://knowrob.org/kb/knowrob.owl#SUFFIX
    if uri:
        return str(uri.split('#')[1])
    return ''