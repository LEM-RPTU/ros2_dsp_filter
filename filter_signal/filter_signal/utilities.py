#!/usr/bin/env python3
import functools
from typing import Dict

#copied from: https://stackoverflow.com/questions/31174295/getattr-and-setattr-on-nested-subobjects-chained-properties/31174427#31174427
# nested_getattr and nested_setattr are drop-in replacements for getattr and setattr, which can also handle dotted attr strings.
def set_nested_Attr(obj: object, attr: str, val):
    pre, _, post = attr.rpartition('.')
    return setattr(get_nested_Attr(obj, pre) if pre else obj, post, val)

def get_nested_Attr(obj: object, attr: str, *args):
    def _getattr(obj, attr):
        return getattr(obj, attr, *args)
    return functools.reduce(_getattr, [obj] + attr.split('.'))


def iterate_leafs(d: Dict, parent_key: str):
# Iterate through the dictionary
    for key, value in d.items():
        # Create the full key path
        full_key = f"{parent_key}.{key}" if parent_key else key
        # If the value is a dictionary, recursively explore it
        if isinstance(value, dict):
            yield from iterate_leafs(value, full_key)  # Recursively go deeper
        else:
            # If it's a leaf, yield the full key and its corresponding value
            yield full_key, value