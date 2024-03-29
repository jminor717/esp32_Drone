# from https://stackoverflow.com/questions/56923895/auto-increment-build-number-using-platformio
import os
import datetime
FILENAME_BUILDNO = 'versioning'
FILENAME_VERSION_H = 'include/buildVersion.h'
version = 'v0.1.'

os.chdir('../')
# print(os.getcwd())

build_no = 0
try:
    with open(FILENAME_BUILDNO) as f:
        build_no = int(f.readline()) + 1
except:
    print('Starting build number from 1..')
    build_no = 1
with open(FILENAME_BUILDNO, 'w+') as f:
    f.write(str(build_no))
    print('Build number: {}'.format(build_no))

hf = """
#ifndef BUILD_NUMBER
  #define BUILD_NUMBER "{0}"
#endif
#ifndef VERSION
  #define VERSION "{1} || {2}"
#endif
#ifndef VERSION_SHORT
  #define VERSION_SHORT "{3}"
#endif
#ifndef BUILD_DATE
  #define BUILD_DATE "{2}"
#endif
""".format(build_no, version+str(build_no), datetime.datetime.now(), version+str(build_no))
with open(FILENAME_VERSION_H, 'w+') as f:
    f.write(hf)
