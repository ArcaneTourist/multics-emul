#!/bin/sh

make || exit $?
renice +8 $$
ulimit -c unlimited
if [ -f simh.log ]; then mv simh.log simh.log.old; fi
if [ -f debug.log ]; then mv debug.log debug.log.old; fi
if [ -f console.log ]; then mv debug.log console.log.old; fi

if [ $# -eq 0 ]; then
	echo "$0: No arguments given.  Consider running as: $0 etc/multics.run.ini" >&2
	sleep 3
fi

set -x
script --flush -c "bin/multics '$@'" simh.log
# MALLOC_CHECK_=3 bin/multics "$@"
# efence bin/multics 
# valgrind bin/multics "$@"
# valgrind --leak-check=full -v bin/multics "$@"
# valgrind --track-origins=yes -v bin/multics "$@"
