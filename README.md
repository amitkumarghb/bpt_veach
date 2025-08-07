### Introduction

A simple C++ bi-directional path tracer based upon the thesis by Eric Veach.

The goal of the code is to be as readable as possible, so not optimisation have been done.

Veach's calculations (chapter 10), have been split into three (3) types, as it furthers the understanding of what the algorithm achieves.

- Type 1)
Complete path traces, which might hit an emitter or camera.
- Type 2)
Connecting all valid path vertices (i.e. non-dirac materials) to emitter or camera.
- Type 3)
Connecting all valid path vertices, between the two paths.

##### Terminology:

| In code | Type | Meaning
| :- | :- | :-
|f_ | prefix | boolean or bit flags
|p_ | prefix | smart pointer, or reference to smart pointer
|ptr_ | prefix | raw pointer (C style)
|pdf_ | prefix | probability density (function)
| _W | suffix | Solid angle
| _A | suffix | Area

### Dependencies

- C++20
- OpenMP (for parallel processing)

### Renders

![Mirror tall block](https://github.com/Thomas-Klietsch/bpt_veach/blob/master/image/mirror.png)

![Diffuse tall block](https://github.com/Thomas-Klietsch/bpt_veach/blob/master/image/lambert.png)
