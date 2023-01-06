#!/usr/bin/env python3.10
# -*- coding: UTF-8 -*-

import unittest
import traceback
import itertools
from typing import Any
from unittest import mock
from unittest.case import _AssertRaisesContext


# From (https://stackoverflow.com/a/49062929)
class ExtendedTestCase(unittest.TestCase):

  class _AssertNotRaisesContext(_AssertRaisesContext):

    def __exit__(self, exc_type, exc_value, tb):
      if exc_type is not None:
        self.exception = exc_value.with_traceback(None)

        try:
          exc_name = self.expected.__name__
        except AttributeError:
          exc_name = str(self.expected)

        if self.obj_name:
          self._raiseFailure("{} raised by {}".format(exc_name, self.obj_name))
        else:
          self._raiseFailure("{} raised".format(exc_name))

      else:
        traceback.clear_frames(tb)

      return True

  def assertNotRaises(self, expected_exception, *args, **kwargs):
    context = ExtendedTestCase._AssertNotRaisesContext(expected_exception, self)
    try:
      return context.handle('assertNotRaises', args, kwargs)
    finally:
      context = None


def mockAttr(target: Any, attribute: str, *, new_callable=None,
             side_effect=None, return_value=mock.DEFAULT, name=None,
             unsafe=False, **kwargs) -> None:
  """Test helper function. Mocks/autospec/spec_set an existing instance function or attribute."""

  assert hasattr(target, attribute)
  assert new_callable == None or callable(getattr(target, attribute))
  assert return_value == None or callable(getattr(target, attribute))
  assert side_effect == None or callable(getattr(target, attribute))
  assert "autospec" not in kwargs
  assert "spec_set" not in kwargs
  assert "create" not in kwargs
  assert "wraps" not in kwargs
  assert "spec" not in kwargs

  mock.patch.object(target, attribute, spec=None, create=False, spec_set=None,
                    autospec=True, new_callable=new_callable,
                    side_effect=side_effect, return_value=return_value,
                    wraps=None, name=name, unsafe=unsafe, **kwargs).start()


def wrapAttr(target: Any, attribute: str, *, new_callable=None,
             side_effect=None, return_value=mock.DEFAULT, wraps=None, name=None,
             unsafe=False, **kwargs) -> None:
  """Test helper function. Mocks/spec_set and wraps an existing instance function."""

  assert hasattr(target, attribute)
  assert callable(getattr(target, attribute))
  assert "autospec" not in kwargs
  assert "spec_set" not in kwargs
  assert "create" not in kwargs
  assert "spec" not in kwargs

  if wraps == None:
    wraps = getattr(target, attribute)

  # Autospec=True + wraps!=None doesn't work
  mock.patch.object(target, attribute, spec=None, create=False, spec_set=True,
                    autospec=None, new_callable=new_callable,
                    side_effect=side_effect, return_value=return_value,
                    wraps=wraps, name=name, unsafe=unsafe, **kwargs).start()
