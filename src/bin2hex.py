#!/usr/bin/env python

import sys

fileout = sys.argv[1]
print fileout
filename = sys.argv[2]
print filename

fr = open(filename, "rb")
fw = open(fileout, "w")


#fw.write("const uint8_t TinyApp[] = {\n") 

st = fr.read(16)

while st:
  for b in st:
    fw.write("0x%02x," % ord(b))
  fw.write("\n")
  st = fr.read(16)

fw.write("\n") 

#fw.write("} ;\n") 


fw.close()

