#!/bin/python3

import sys
import os
import types

sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), 'python_deps'))

# The checked-in dependency set contains a Python 3.9 rpds binary. Keep full
# schema validation on compatible Python versions, but allow the frozen ODrive
# interface definition to generate on newer Python runtimes.
try:
    import jsonschema  # noqa: F401
except ImportError as ex:
    print("Warning: JSON schema validation disabled: " + str(ex), file=sys.stderr)

    class _FallbackDraft4Validator:
        def __init__(self, *args, **kwargs):
            pass

        def iter_errors(self, instance):
            return ()

    jsonschema_fallback = types.ModuleType('jsonschema')
    jsonschema_fallback.Draft4Validator = _FallbackDraft4Validator
    sys.modules['jsonschema'] = jsonschema_fallback

try:
    exec(open(os.path.join(os.path.dirname(os.path.realpath(__file__)), 'fibre', 'tools', 'interface_generator.py')).read())
except ImportError as ex:
    print(str(ex), file=sys.stderr)
    print("Note that there are new compile-time dependencies since around v0.5.1.", file=sys.stderr)
    print("Check out https://github.com/madcowswe/ODrive/blob/devel/docs/developer-guide.md#prerequisites for details.", file=sys.stderr)
    exit(1)
