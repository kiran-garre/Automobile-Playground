# Automobile Playground

Welcome to my virtual car! Here you'll find 5 files that each represent the most important components of a car: the crankshaft and cylinders, the engine (a wrapper for the former), the torque converter, the (automatic) transmission, and the wheels/rest of the car. A 6th file is also included as a wrapper class for the rest of the parts. The car has over 15 parameters that you can choose to make whatever type of engine you desire. If that isn't enough for you, you can always go into the source code itself to change whatever physics or properties you'd like!

### Why I made this simulation:
Let's face it: cars are not easy to play with. The lack of tools, mechanical knowledge, and budget make it difficult to tinker with the innards of the modern automobile. Deep inside, we all want to stick the world's largest, most powerful engine in the common Honda Civic and see what happens, but unless you have a team of professional engineers and a multi-million dollar budget, our dreams will go unrealized.

Unfortunately, I have neither of those, so I had to resort to a cheaper option. In my simulation, you can change a multitude of parameters and see how they affect the car's performance in real-time. If you want to put a gas guzzling, two-thousand-horsepower monster inside a minivan, be my guest. If you feel the urge to see how a modern sports car would perform with the Ford Model T's engine, go for it. The possibilities are endless.

### What this simulation is:
This simulation provides an approximation of a car's performance given reasonable paramters. Almost any value can be read and/or changed at any time, giving you full control over the virtual car. 

### What this simulation is not:
This simulation is NOT a perfect, physically accurate simulation of a car. The physics are simplified, values change instantaneously, and almost every property is an approximation of its real-life counterpart. My simulation is only meant to be a quick approximation of the car's overall _behavior,_ not exactly how the car would function in the real world. There is no guarantee that the same level of accuracy will hold across every possible configuration. 

### Getting started:
Simply use the `parameters.txt` file to change the engine however you'd like (leaving a space blank will use a default value). Use a `Car` object's `demo_run()` method to get an idea of how the car works. To experiment further, you'll want to use the `update()` function with your own code to change paramters on the fly.



