#! /usr/bin/env python
import traceback, pdb, sys, hotshot, os

def pdbwrap(f):
    '''A utility for dropping out to a debugger on exceptions.'''
    def fdebug(*a, **kw):
        try:
            return f(*a, **kw)
        except Exception:
            print 
            type, value, tb = sys.exc_info()
            traceback.print_exc(file=sys.stderr)
            os.system('stty sane')
            
            if sys.stdin.isatty():
                pdb.post_mortem(tb)
            else:
                sys.exit(1)
    return fdebug
