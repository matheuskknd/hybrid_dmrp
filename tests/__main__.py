#!/usr/bin/env python3.10
# -*- coding: UTF-8 -*-

# Ugly hack to allow absolute import from the root folder
# whatever its name is. Please forgive the heresy.
if __name__ == "__main__":
  from sys import path
  from os.path import (dirname, join)

  parentFolder: str = dirname(dirname(__file__))
  path.insert(0, parentFolder)

  # Tests won't work otherwise
  assert join(parentFolder, "tests", "__main__.py") == __file__

################################
########## Main Start ##########
################################

import unittest

from .test_LocalBranching import *

# Runs the program
if __name__ == "__main__":
  unittest.main()
