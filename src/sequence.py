"""Pickle-compatibility shim -- do not delete or rename.

expctl clients pickle their ``Sequence`` objects from a top-level module named
``sequence``, so ``pickle.loads`` on this server looks the classes up under
exactly that module name. The minimal class definitions live in
``ServerClass.py`` (see "Sequence pickle shells" there); this module only
re-exports them under the name recorded in the pickles.
"""

from ServerClass import Sequence, Channel, Interval, id_trans  # noqa: F401
