# Quadruped in OMPL

Implementation of the quadruped in OMPL. It has been treated as a unicycle. Primitives:
- rolling with 4 wheels (fw/bw/l/r)
- rolling with 2 wheels (fw/bw)
- turning on the spot
- stepping with one foot (fw/bw)

A gap is considered (both finite and infinite).

To be done:
- introduce obstacles if needed

*Video 1*: each foot can step everywhere, infinite gap <br/>
*Video 2*: a foot can step only if it is in the green area, infinite gap <br/>
*Video 3*: a foot can step only if it is in the green area, finite gap (the robot steps with all the four feet) <br/
*Video 4*: a foot can step only if it is in the green area, finite gap (the robot steps with only the two back feet) <br/>
