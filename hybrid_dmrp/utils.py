#!/usr/bin/env python3.10
# -*- coding: UTF-8 -*-

from typing import Any


def extended_enum_new(enumClass: type) -> callable:
  """
    It returns a replacemnet for an enumaration __new__ method
    that is capable of creating a valid enum value from its name,
    value, or even from the own enum value itself.
  """

  def __new__(cls: type, value: Any):
    try:
      return vars(cls)[value]

    except:
      return enumClass.__new__(cls, value)

  return __new__
