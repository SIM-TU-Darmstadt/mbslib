mbstestlib
----------

example usage: testsuite testlist.txt (will write log to 'testlog.txt')

The testlog.txt log can be converted into a html table via the
'testlog2HTMLConverter.py' script.

Testsets can be specified either in an intermediate form (see 'testfile-hints.txt')
or via an xml format (see 'testfile-example.xml'). The latter has to be
converted into intermediate via the 'testsetXML2intermediateConverter.py'
script.

The tests are stored in tests\ subdir. Each testfile has to be in intermediate format
and be listed in the 'testlist.txt' file. (e.g. don't forget to add new
tests there)