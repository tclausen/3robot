import sys
import os

class Log:
    dirname = "logs"
    
    def __init__(self, t0, filename):
        if not os.path.exists(Log.dirname):
            os.makedirs(Log.dirname)
        fullFilename = os.path.join(Log.dirname, filename)
        if os.path.exists(fullFilename):
            os.remove(fullFilename)
        self.f = open(fullFilename, 'a')
        self.t0 = t0
        
    def log(self, text):
        self.f.write(text + "\n")

    def logt(self, t, text):
        self.f.write("%f %s\n" % (t-self.t0, text))
    
    def close(self):
        self.f.close()
        
