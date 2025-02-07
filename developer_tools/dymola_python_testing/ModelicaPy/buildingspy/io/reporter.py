#Licensed under Apache 2.0 License.
#© 2020 Battelle Energy Alliance, LLC
#ALL RIGHTS RESERVED
#.
#Prepared by Battelle Energy Alliance, LLC
#Under Contract No. DE-AC07-05ID14517
#With the U. S. Department of Energy
#.
#NOTICE:  This computer software was prepared by Battelle Energy
#Alliance, LLC, hereinafter the Contractor, under Contract
#No. AC07-05ID14517 with the United States (U. S.) Department of
#Energy (DOE).  The Government is granted for itself and others acting on
#its behalf a nonexclusive, paid-up, irrevocable worldwide license in this
#data to reproduce, prepare derivative works, and perform publicly and
#display publicly, by or on behalf of the Government. There is provision for
#the possible extension of the term of this license.  Subsequent to that
#period or any extension granted, the Government is granted for itself and
#others acting on its behalf a nonexclusive, paid-up, irrevocable worldwide
#license in this data to reproduce, prepare derivative works, distribute
#copies to the public, perform publicly and display publicly, and to permit
#others to do so.  The specific term of the license can be identified by
#inquiry made to Contractor or DOE.  NEITHER THE UNITED STATES NOR THE UNITED
#STATES DEPARTMENT OF ENERGY, NOR CONTRACTOR MAKES ANY WARRANTY, EXPRESS OR
#IMPLIED, OR ASSUMES ANY LIABILITY OR RESPONSIBILITY FOR THE USE, ACCURACY,
#COMPLETENESS, OR USEFULNESS OR ANY INFORMATION, APPARATUS, PRODUCT, OR
#PROCESS DISCLOSED, OR REPRESENTS THAT ITS USE WOULD NOT INFRINGE PRIVATELY
#OWNED RIGHTS.
#!/usr/bin/env python


class Reporter:
    ''' Class that is used to report errors.
    '''

    def __init__(self, fileName):
        ''' Construct a reporter.

        :param fileName: Name of the output file.

        This class writes the standard output stream and the
        standard error stream to the file ``fileName``.
        '''
        import os

        self._logToFile = True
        self._verbose = True
        self._iWar = 0
        self._iErr = 0
        self.logToFile()
        self._logFil = os.path.join(fileName)
        self.deleteLogFile()

    def deleteLogFile(self):
        ''' Deletes the log file if it exists.
        '''
        import os
        if os.path.isfile(self._logFil):
            os.remove(self._logFil)

    def logToFile(self, log=True):
        ''' Function to log the standard output and standard error stream to a file.

        :param log: If ``True``, then the standard output stream and the standard error stream will be logged to a file.

        This function can be used to enable and disable writing outputs to
        the file ''fileName''.
        The default setting is ``True``
        '''
        self._logToFile = log

    def getNumberOfErrors(self):
        ''' Returns the number of error messages that were written.

        :return : The number of error messages that were written.
        '''
        return self._iErr

    def getNumberOfWarnings(self):
        ''' Returns the number of warning messages that were written.

        :return : The number of warning messages that were written.
        '''
        return self._iWar

    def writeError(self, message):
        ''' Writes an error message.

        :param message: The message to be written.

        Note that this method adds a new line character at the end of the message.
        '''
        self._iErr += 1
        self._writeErrorOrWarning(True, message)
        return

    def writeWarning(self, message):
        ''' Writes a warning message.

        :param message: The message to be written.

        Note that this method adds a new line character at the end of the message.
        '''
        self._iWar += 1
        self._writeErrorOrWarning(False, message)
        return

    def _writeErrorOrWarning(self, isError, message):
        ''' Writes an error message or a warning message.

        :param isError: Set to 'True' if an error should be written, or 'False' for a warning.
        :param message: The message to be written.

        Note that this method adds a new line character at the end of the message.
        '''
        import sys

        msg = ""
        if self._verbose:
            if isError:
                msg += "*** Error: "
            else:
                msg += "*** Warning: "
        msg += message + "\n"
        sys.stderr.write(msg)
        if self._logToFile:
            fil = open(self._logFil, 'a')
            fil.write(msg)
            fil.close()
        return

    def writeOutput(self, message):
        ''' Writes a message to the standard output.

        :param message: The message to be written.

        Note that this method adds a new line character at the end of the message.
        '''
        import sys

        msg = message + "\n"
        if self._logToFile:
            fil = open(self._logFil, 'a')
            fil.write(msg)
            fil.close()
        sys.stdout.write(msg)
        return

