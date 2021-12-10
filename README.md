The aim of the project is to develop a multi–agent simulation based on Boids model for conserving wildlife, the simulation consists of the following three main agents: preys, poachers and rangers. The boids are differentiated using multiple coloured polygons. The simulation also consists of attraction points which act as
resources to attract the preys to move closer to them.
• White triangles: Preys
• Red triangles: Poachers
• Green triangles: Rangers
• Yellow triangles: Drones
• Blue circles: Attraction points

As the project aims to develop a multi–agent simulation based on spatio–temporal modelling for wildlife conservation, Boids model aligns well with the project objectives. This project is an extension of Michael Dodsworth work on Boids model using single entity. From there, we have expanded the simulation by introducing multiple classes to represent the different agents and their behaviours.
We also coded the simulation to recreate natural phenomenons of wildlife: 
i) Implementation of self–reproducing dynamics based on autopoiesis principles on preys, 
ii) Poachers moving in preys direction to attack them
iii) Rangers detecting the poachers position and attack them before a poacher–prey attack,
iv) Drones detecting poachers coordinates and update the rangers to re-direct them towards the poachers
iv) More rangers appearing in the environment if the poachers population increases and 
v) More poachers appearing in the environment if the preys reproduction rate is high.

The agents population is recorded at every delta time(in seconds) and saved in a comma–separated values(CSV) file. The population growth graph is generated at
the end of every simulation run. Pandas, an open source, BSD-licensed library was used together with Pyglet to plot the graphs. These graphs are used to perform
statistical data analysis on species populations and study their spatial patterns.
