#!C:\Users\gps\AppData\Local\Temp\python-build-as6__1b1\out\python\install\python.exe
# EASY-INSTALL-ENTRY-SCRIPT: 'pip==20.0.2','console_scripts','pip'
__requires__ = 'pip==20.0.2'
import re
import sys
from pkg_resources import load_entry_point

if __name__ == '__main__':
    sys.argv[0] = re.sub(r'(-script\.pyw?|\.exe)?$', '', sys.argv[0])
    sys.exit(
        load_entry_point('pip==20.0.2', 'console_scripts', 'pip')()
    )
