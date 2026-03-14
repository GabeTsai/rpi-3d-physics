.include "../share/vc4inc/vc4.qinc"

nop     # page 37 - a scoreboard wait must not occur in the first two instructions
nop     # of a fragment shader

sbwait 

# to perform the Z test, you must read Z data from rb15 and then write it to TLB_Z
nop         # 2-instruction gap required between sbwait and first TLB write
read rb15   # page 51 - read z data
mov tlbz, rb15  # page 47 - write z value to TLB_Z I/O register

ldi r0, 0xffff0000 # assuming rgba8888, this is red

mov tlbc, r0 # write color to tile buffer

sbdone # release scoreboard

thrend # program end
nop 
nop
