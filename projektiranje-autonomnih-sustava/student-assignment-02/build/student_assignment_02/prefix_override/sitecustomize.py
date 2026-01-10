import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ivan/Documents/PAS/zadatak2_oroginal_novo/projektiranje-autonomnih-sustava/student-assignment-02/install/student_assignment_02'
