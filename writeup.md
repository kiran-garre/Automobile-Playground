Many years ago, I was an avid player of racing games. Games where I could burn down the highway at 250 miles an hour, plowing through other cars with no damage to my own were the most entertaining.

There was one problem with these games: the customization. I could choose what car I wanted, what upgrades it had, and its paint color, but nothing else. But what I couldn't do was swap engines, transmissions, or even the wheels. 

As I grew older, these creativity-killing restrictions bugged me. What was the point of customization if it was so contstrained?

This question was the resaon I made this program. I wanted real customization, not the ability to buy a paint job. If I wanted to change its engine and observe the effects, i could. If I wanted to floor the virtual gas and get
real-time stats on its torque and horsepower, I could. It would be a completely transparent, fully maleable virtual car. 

I thought to myself: where should I start? Theoretically, I could make it as deep as coding each individual atom and their interactions or as surface-level as increasing the car's speed based on some mathematical function. The former 
was too difficult, and the latter was unsatisfactory. 

I decided to choose somewhere in between: a single cylinder within an engine.

```
class Cylinder
```
What parameters would I need? I started with the bore and stroke (and the resulting volume of the cylinder). The position of the piston within the cylinder would also be important, as well as the piston's current stroke.
(Note: the Cylinder class represents the piston and cylinder together.)

The basic premise behind a cylinder is the combustion of gasoline vapor creates immense pressure, pushing on the piston to generate a force. Thus, my next step was to code the 4 strokes: intake, compression, power, and exhaust.
```
def intake(self):
  pass
def spark(self):
  pass
def exhaust(self):
  pass
```
You'll notice that there is no function for compression. This is because, during the second stroke, the compression is natural result of the the piston pushing the gas vapor into a tighter area. There doesn't need to be an extra 
function because there is no special behavior happening on the second stroke.

Of course, wihout any oscillatory motion, these strokes didn't mean anything. The most I could do was fill the cylinder with gas, ignite it, and have the piston move in one direction for the length of the stroke. This was boring,
and it was hard to know how well it was working.

To remedy this, I decided to focus on the next most imporant part of the engine: the crankshaft.
```
class Crankshaft
```
