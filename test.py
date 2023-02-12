import os, sys
try:
    sys.exit(0) 
except:
    print('die')
finally:
    print('cleanup') 
    
    
try:
    os._exit(0) 
except:
    print('die')
    print('os.exit') #不列印直接退出