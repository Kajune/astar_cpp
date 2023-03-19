# -*- coding: utf-8 -*-
import os
import platform
import subprocess
import sys
from distutils.core import setup, Extension


ex_module = Extension(
	'astar_cpp',
	sources=['astar.cpp'],
	libraries=['boost_python', 'boost_numpy'],
	include_dirs=['/usr/include', '/usr/local/include'],
	library_dirs=['/usr/lib/x86_64-linux-gnu', '/usr/local/lib/x86_64-linux-gnu'],
	extra_compile_args=['-std=c++2a', '-O3'],
)

setup(name="astar_cpp", version="1.0", ext_modules=[ex_module])
