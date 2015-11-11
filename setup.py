from distutils.core import setup, Extension

module1 = Extension('dgtpi',
                    sources = ['dgtpi.c'])

setup (name = 'DGTPi',
       version = '1.0',
       description = 'DGTPi controlling system',
       ext_modules = [module1])