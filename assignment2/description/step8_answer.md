CS 348B Assignment 2

Problem / trouble description
1.	The first trouble I ran into is I updated the tHit and isect even when the two are NULL. I posted a question on Piazza and was instructed to read the instruction more closely. I used Xcode to debug and confirmed it won’t be updated.
2.	Another trouble is I found a hard time passing the value of vector3f data in the code. I looked the pbrt file and follow the format and successfully pass the data in the code.
3.	The pure virtual function and the virtual function confused me for a long time. It turned that the pure virtual function has set “const = 0”, which means it need to be defined in the subclass. For the virtual function, it will be defined in the .cpp file.
