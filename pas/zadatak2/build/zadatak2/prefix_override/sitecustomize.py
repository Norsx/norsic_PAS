import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ivan/Documents/PAS/zadatak2/pas/zadatak2/install/zadatak2'
