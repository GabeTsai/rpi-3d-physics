.include "../share/vc4inc/vc4.qinc"

nop     # page 37 - a scoreboard wait must not occur in the first two instructions
nop     # of a fragment shader

sbwait 

ldi r0, 0xff0000ff # assuming rgba8888, this is red

mov tlbc, r0 # write color to tile buffer

sbdone # release scoreboard

thrend # program end
nop 
nop
