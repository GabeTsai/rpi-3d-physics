.include "../share/vc4inc/vc4.qinc"

nop     # page 37 - a scoreboard wait must not occur in the first two instructions
nop     # of a fragment shader

sbwait 

# to perform the Z test, you must read Z data from rb15 and then write it to TLB_Z
nop         # 2-instruction gap required between sbwait and first TLB write
read rb15   # page 51 - read z data
mov tlbz, rb15  # page 47 - write z value to TLB_Z I/O register

# -- R -- 
mov r0, vary
fmul r0, r0, ra15 # page 51 - V * W + C, where ra15 is W
fadd r0, r0, r5 # page 51 - C is auto stored in r5

# -- G -- 
mov r1, vary
fmul r1, r1, ra15
fadd r1, r1, r5 

# -- B -- 
mov r2, vary
fmul r2, r2, ra15
fadd r2, r2, r5 

# scale to [0, 255] - 255.0 can't fit in small immediate, load into register first
ldi r3, 255.0
fmul r0, r0, r3
fmul r1, r1, r3
fmul r2, r2, r3

ftoi r0, r0
ftoi r1, r1
ftoi r2, r2

shl r1, r1, 8
shl r0, r0, 16
ldi r3, 0xff000000

or r0, r0, r1
or r0, r0, r2
or r0, r0, r3

mov tlbc, r0 # write color to tile buffer

sbdone # release scoreboard

thrend # program end
nop 
nop
